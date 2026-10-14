"""Contact and signed-clearance audit for the active HEAL + Robotiq scene.

This is passive measurement. It does not change controls or enforce avoidance.
Distances are queried with MuJoCo's mj_geomDistance for each monitored pair at
every simulation step, while contact events come from mjData.contact.
"""
from __future__ import annotations

from collections import defaultdict

import mujoco
import numpy as np


PHASES = ("approach", "descend", "close_gripper", "lift", "transfer",
          "lower_place", "release", "retreat")
EXPECTED_PAD_PHASES = {"close_gripper", "lift", "transfer", "lower_place", "release"}
EXPECTED_CUBE_SUPPORT_PHASES = {"approach", "descend", "close_gripper", "lower_place", "release", "retreat"}


def _name(model, geom_id):
    named = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id))
    if named:
        return named
    body_id = int(model.geom_bodyid[int(geom_id)])
    body = _body_name(model, body_id)
    if int(model.geom_type[int(geom_id)]) == int(mujoco.mjtGeom.mjGEOM_MESH):
        mesh_id = int(model.geom_dataid[int(geom_id)])
        mesh = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id) or "mesh"
        return f"{body}_{mesh}_collision"
    return f"{body}_geom_{geom_id}"


def _body_name(model, body_id):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(body_id)) or f"body_{body_id}"


def _is_descendant(model, body_id, ancestor_name):
    wanted = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ancestor_name)
    current = int(body_id)
    while current >= 0:
        if current == wanted:
            return True
        if current == 0:
            break
        current = int(model.body_parentid[current])
    return False


class CollisionMonitor:
    """Track contacts and exact signed geom distances for task-relevant pairs."""

    def __init__(self, model, distmax_m=10.0):
        self.model = model
        self.distmax_m = float(distmax_m)
        self.steps = 0
        self.distance_query_count = 0
        self.min_by_pair = {}
        self.min_by_category = {}
        self.min_unexpected_by_pair = {}
        self.min_unexpected_by_category = {}
        self.expected_contact_observations = 0
        self.unexpected_contact_observations = 0
        self.expected_contact_pairs = set()
        self.unexpected_contact_pairs = set()
        self.actual_contact_pairs = set()
        self.penetrating_contact_observations = 0
        self.unexpected_actual_contact_observations = 0
        self.unexpected_penetrating_contact_observations = 0
        self.deepest_penetration_m = 0.0
        self.deepest_unexpected_penetration_m = 0.0
        self._phase_unexpected = defaultdict(int)
        self._phase_expected = defaultdict(int)
        self._contact_stats = defaultdict(lambda: {"phase_observations": defaultdict(int),
                                                    "minimum_contact_distance_m": float("inf"),
                                                    "actual_contact_observations": 0})
        self._last_phase = None
        self.last_unexpected_minimum_distance_m = None
        self.last_unexpected_minimum_pair = None
        self._last_actual_contact_set = set()
        self._contact_event_counts = defaultdict(int)
        self._contact_event_pairs = defaultdict(set)
        self.geometry = self._resolve_geometry()
        self.pairs = self._resolve_pairs()
        self.pair_by_ids = {tuple(sorted(pair["geom_ids"])): pair for pair in self.pairs}

    def _resolve_geometry(self):
        m = self.model
        geom_ids = [g for g in range(m.ngeom)
                    if int(m.geom_contype[g]) != 0 or int(m.geom_conaffinity[g]) != 0]
        table_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "worktable")
        floor_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        cube_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
        if min(table_id, floor_id, cube_id) < 0:
            raise ValueError("Expected worktable, floor, and cube_geom collision geoms.")
        arms, gripper = [], []
        arm_names = {"base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "end_effector"}
        for geom_id in geom_ids:
            body_id = int(m.geom_bodyid[geom_id])
            body = _body_name(m, body_id)
            if body in arm_names:
                arms.append(geom_id)
            elif _is_descendant(m, body_id, "gripper_base"):
                gripper.append(geom_id)
        pads = {g for g in gripper if _name(m, g) in {"left_pad1", "left_pad2", "right_pad1", "right_pad2"}}
        finger_surfaces = pads | {g for g in gripper if _name(m, g) in {
            "left_follower_tongue_collision", "right_follower_tongue_collision"}}
        return {"active": geom_ids, "arm": arms, "gripper": gripper,
                "pads": pads, "finger_surfaces": finger_surfaces,
                "table": table_id, "floor": floor_id, "cube": cube_id,
                "tray": None}

    def _resolve_pairs(self):
        geo = self.geometry
        pairs = []

        def add(a, b, role_a, role_b):
            ids = tuple(sorted((int(a), int(b))))
            if ids[0] == ids[1] or any(p["geom_ids"] == ids for p in pairs):
                return
            pairs.append({"geom_ids": ids, "roles": tuple(sorted((role_a, role_b)))})

        for role in ("arm", "gripper"):
            for g in geo[role]:
                add(g, geo["table"], role, "table")
                add(g, geo["floor"], role, "floor")
                add(g, geo["cube"], role, "cube")
        add(geo["cube"], geo["table"], "cube", "table")
        add(geo["cube"], geo["floor"], "cube", "floor")
        return pairs

    def _expected(self, pair, phase, data=None):
        ids = set(pair["geom_ids"])
        if ids == {self.geometry["cube"], self.geometry["table"]}:
            if phase in EXPECTED_CUBE_SUPPORT_PHASES:
                return True, "cube-table support is expected while resting on the table"
            # During commanded lift, support can persist briefly until the
            # object clears the tabletop. A later transfer-phase landing is
            # treated as an unexpected loss of clearance.
            if phase == "lift" and data is not None:
                cube_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "cube")
                table_top = (data.geom_xpos[self.geometry["table"]][2]
                             + self.model.geom_size[self.geometry["table"]][2])
                cube_half_z = self.model.geom_size[self.geometry["cube"]][2]
                if data.xpos[cube_body][2] <= table_top + cube_half_z + 0.006:
                    return True, "brief cube support contact while beginning lift"
            return False, "cube-table contact during lift after clearance or during transfer is unexpected"
        finger_ids = self.geometry["finger_surfaces"]
        if (self.geometry["cube"] in ids and len(ids & finger_ids) == 1
                and phase in EXPECTED_PAD_PHASES):
            return True, "cube-fingertip/pad/tongue contact is expected during grasp/carry/place"
        return False, ""

    def update(self, data, phase_index):
        phase = PHASES[phase_index]
        self.steps += 1
        self._last_phase = phase
        category_min = {}
        unexpected_minimum = float("inf")
        unexpected_minimum_pair = None
        for pair in self.pairs:
            g1, g2 = pair["geom_ids"]
            distance = float(mujoco.mj_geomDistance(self.model, data, g1, g2,
                                                    self.distmax_m, None))
            self.distance_query_count += 1
            names = tuple(sorted((_name(self.model, g1), _name(self.model, g2))))
            key = "<->".join(names)
            old = self.min_by_pair.get(key)
            if old is None or distance < old["minimum_signed_distance_m"]:
                self.min_by_pair[key] = {
                    "geom_pair": names,
                    "roles": pair["roles"],
                    "minimum_signed_distance_m": distance,
                    "phase_at_minimum": phase,
                }
            category = "-".join(pair["roles"])
            category_min[category] = min(category_min.get(category, float("inf")), distance)
            expected, _ = self._expected(pair, phase, data)
            if not expected:
                if distance < unexpected_minimum:
                    unexpected_minimum = distance
                    unexpected_minimum_pair = key
                old_unexpected = self.min_unexpected_by_pair.get(key)
                if old_unexpected is None or distance < old_unexpected["minimum_signed_distance_m"]:
                    self.min_unexpected_by_pair[key] = {
                        "geom_pair": names,
                        "roles": pair["roles"],
                        "minimum_signed_distance_m": distance,
                        "phase_at_minimum": phase,
                    }
                self.min_unexpected_by_category[category] = min(
                    self.min_unexpected_by_category.get(category, float("inf")), distance)
        for category, value in category_min.items():
            self.min_by_category[category] = min(self.min_by_category.get(category, float("inf")), value)

        actual_contact_set = set()
        contact_event_meta = {}
        for ci in range(data.ncon):
            contact = data.contact[ci]
            ids = tuple(sorted((int(contact.geom1), int(contact.geom2))))
            pair = self.pair_by_ids.get(ids)
            if pair is None:
                continue
            expected, _ = self._expected(pair, phase, data)
            names = tuple(sorted((_name(self.model, ids[0]), _name(self.model, ids[1]))))
            pair_key = "<->".join(names)
            stats = self._contact_stats[pair_key]
            stats["phase_observations"][phase] += 1
            stats["minimum_contact_distance_m"] = min(stats["minimum_contact_distance_m"], float(contact.dist))
            if float(contact.dist) <= 0.0:
                self.actual_contact_pairs.add(pair_key)
                stats["actual_contact_observations"] += 1
                actual_contact_set.add(pair_key)
                contact_event_meta[pair_key] = expected
            if float(contact.dist) < -1e-6:
                self.penetrating_contact_observations += 1
                self.deepest_penetration_m = min(self.deepest_penetration_m, float(contact.dist))
                if not expected:
                    self.unexpected_penetrating_contact_observations += 1
                    self.deepest_unexpected_penetration_m = min(
                        self.deepest_unexpected_penetration_m, float(contact.dist))
            if expected:
                self.expected_contact_observations += 1
                self.expected_contact_pairs.add(pair_key)
                self._phase_expected[phase] += 1
            else:
                self.unexpected_contact_observations += 1
                self.unexpected_contact_pairs.add(pair_key)
                self._phase_unexpected[phase] += 1
                if float(contact.dist) <= 0.0:
                    self.unexpected_actual_contact_observations += 1

        # Count contact bouts as transitions into actual contact, separate from
        # per-step observations. A continuously touching pair is one event.
        for pair_key in actual_contact_set - self._last_actual_contact_set:
            expected = contact_event_meta[pair_key]
            category = "expected" if expected else "unexpected"
            self._contact_event_counts[category] += 1
            self._contact_event_pairs[category].add(pair_key)
        self._last_actual_contact_set = actual_contact_set
        self.last_unexpected_minimum_distance_m = (
            None if unexpected_minimum == float("inf") else unexpected_minimum
        )
        self.last_unexpected_minimum_pair = unexpected_minimum_pair

    def summary(self):
        minima = sorted(self.min_by_pair.values(), key=lambda x: x["minimum_signed_distance_m"])
        return {
            "measurement_scope": "post-step scene monitor; exact mj_geomDistance query for each listed geom pair at every physics step",
            "distance_sign_definition": "positive=separated; zero=touching; negative=geometric overlap/penetration",
            "steps_monitored": self.steps,
            "distance_queries": self.distance_query_count,
            "distmax_m": self.distmax_m,
            "active_collision_geom_count": len(self.geometry["active"]),
            "arm_collision_geoms": [_name(self.model, g) for g in self.geometry["arm"]],
            "gripper_collision_geoms": [_name(self.model, g) for g in self.geometry["gripper"]],
            "expected_finger_contact_geoms": sorted(_name(self.model, g) for g in self.geometry["finger_surfaces"]),
            "cube_geom": _name(self.model, self.geometry["cube"]),
            "table_geom": _name(self.model, self.geometry["table"]),
            "floor_geom": _name(self.model, self.geometry["floor"]),
            "tray_geom": None,
            "monitored_pair_count": len(self.pairs),
            "min_signed_distance_by_role_pair_m": self.min_by_category,
            "minimum_pairs_m": minima,
            "min_unexpected_signed_distance_by_role_pair_m": self.min_unexpected_by_category,
            "minimum_unexpected_pairs_m": sorted(self.min_unexpected_by_pair.values(),
                                                  key=lambda x: x["minimum_signed_distance_m"]),
            "actual_contact_pairs": sorted(self.actual_contact_pairs),
            "distinct_contact_event_counts": dict(self._contact_event_counts),
            "distinct_contact_event_pairs": {
                category: sorted(pairs) for category, pairs in self._contact_event_pairs.items()
            },
            "contact_observation_definition": "one pair observation per physics step with a MuJoCo contact record",
            "distinct_contact_event_definition": "transition into dist<=0 for a geom pair after a step without that pair in actual contact",
            "contact_pair_details": {
                pair: {"phase_observations": dict(v["phase_observations"]),
                       "minimum_contact_distance_m": v["minimum_contact_distance_m"],
                       "actual_contact_observations": v["actual_contact_observations"]}
                for pair, v in self._contact_stats.items()
            },
            "expected_contact_exclusions": {
                "cube_geom<->worktable": "allowed during approach, descend, close_gripper, lower_place, release, and retreat; during lift only while the cube remains within 6 mm of its geometric support height; transfer-phase table contact is unexpected",
                "cube_geom<->finger surfaces":
                    "left/right pad boxes and left/right follower tongue meshes; allowed phases close_gripper, lift, transfer, lower_place, release; counted separately",
                "tray": "no tray geometry exists; no tray contacts or clearances are measured",
            },
            "expected_contact_observations": self.expected_contact_observations,
            "expected_contact_pairs": sorted(self.expected_contact_pairs),
            "unexpected_contact_observations": self.unexpected_contact_observations,
            "unexpected_contact_pairs": sorted(self.unexpected_contact_pairs),
            "unexpected_contact_observations_by_phase": dict(self._phase_unexpected),
            "expected_contact_observations_by_phase": dict(self._phase_expected),
            "penetrating_contact_observations_lt_minus_1e_6_m": self.penetrating_contact_observations,
            "deepest_contact_penetration_m": self.deepest_penetration_m,
            "unexpected_actual_contact_observations_dist_le_0": self.unexpected_actual_contact_observations,
            "unexpected_penetrating_contact_observations_lt_minus_1e_6_m": self.unexpected_penetrating_contact_observations,
            "deepest_unexpected_penetration_m": self.deepest_unexpected_penetration_m,
            "waypoint_prechecks": False,
            "trajectory_contacts_checked_after_each_simulation_step": True,
            "collision_avoidance_enforced_by_controller": False,
        }
