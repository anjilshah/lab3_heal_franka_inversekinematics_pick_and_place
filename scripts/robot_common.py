"""Shared helpers for the HEAL arm + Robotiq model used in this Lab 3 setup."""
from pathlib import Path
import mujoco

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "robot_descriptions" / "arm_gripper.xml"
ARM_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
ARM_ACTUATORS = ("turret", "shoulder", "elbow", "wrist_1", "wrist_2", "wrist_3")
EE_SITE = "right_center"
GRIPPER_ACTUATOR = "gripper"


def arm_indices(model):
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ARM_JOINTS]
    actuator_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ARM_ACTUATORS]
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, EE_SITE)
    grip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, GRIPPER_ACTUATOR)
    if min(joint_ids + actuator_ids + [site_id, grip_id]) < 0:
        raise ValueError("Expected arm/gripper joints, actuators, or end-effector site are missing.")
    qpos_ids = [model.jnt_qposadr[j] for j in joint_ids]
    dof_ids = [model.jnt_dofadr[j] for j in joint_ids]
    return qpos_ids, dof_ids, actuator_ids, site_id, grip_id


def load_model():
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"Cannot find {MODEL_PATH}. Run scripts from your Lab3 project folder.")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    qpos_ids, dof_ids, actuator_ids, site_id, grip_id = arm_indices(model)
    return model, data, qpos_ids, dof_ids, actuator_ids, site_id, grip_id
