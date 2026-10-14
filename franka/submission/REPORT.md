# Franka Panda Pick-and-Place in MuJoCo

## Objective and setup

This project adapts the pick-and-place assignment to a Franka Panda with seven revolute arm joints and a coupled two-finger hand. The model defines seven arm position actuators and one tendon gripper actuator. The gripper command is 255=open and 0=closed, consistent with the actuator force law in the bundled model.

The table is 0.76 × 0.96 m with its top at z=0.375 m. The cube is a 0.05 m box (0.08 kg) resting at z=0.400 m. The nominal start and destination centers are (0.18, -0.16, 0.400) m and (0.18, 0.16, 0.400) m in the MuJoCo world frame. The Panda base transform and robot dimensions come from the bundled MJCF. MuJoCo 3.15.0, Mink 1.3.0, qpsolvers/DAQP, NumPy, and Matplotlib were available in the parent repository's Python 3.10 environment.

## IK methods and measured batch comparison

All methods used the same 25 seed-derived cube starts, destination coordinates, initial Panda configuration, orientation target, and eight phase task. Success requires the cube to lift more than 45 mm, finish within 10 mm horizontally of the destination, and settle with final z in the open interval (0.39, 0.49) m. The saved nominal DLS, Mink, and QP episodes and all 75 matched batch records were re-evaluated against this criterion; their trajectories were not rerun. DLS uses damped Jacobian updates and does not enforce a hard joint velocity limit. Mink solves a differential frame task with configuration and 1.2 rad/s velocity limits through DAQP. The custom QP uses DAQP to solve a bounded weighted least-squares update, with 1.2 rad/s per-joint update bounds.

| Method | Episodes | Successes | Mean target error (m) | Mean total IK solve time (s) | Mean max position residual (m) | Mean max orientation residual (rad) |
|---|---:|---:|---:|---:|---:|---:|
| DLS | 25 | 25 (100%) | 0.00228 ± 0.00070 | 0.00497 ± 0.00040 | 0.01050 | 0.06731 |
| Mink | 25 | 1 (4%) | 0.04337 ± 0.01261 | 0.09177 ± 0.02043 | 0.01897 | 0.11738 |
| QP | 25 | 5 (20%) | 0.02280 ± 0.01108 | 0.01491 ± 0.00079 | 0.01934 | 0.11783 |

Values are mean ± sample standard deviation for final target error and total IK wall time; residual columns are batch means. Success rates use the 10 mm final XY tolerance along with the retained lift and final-height checks. DLS had the lowest mean solve time and final placement error in this implementation. Episode simulation time was 16 s for every method, based on eight 2 s phase durations.

The raw matched records, manifest, summary, and plots are in [`results/batch_final/`](../results/batch_final/). The batch contains 75 rows, 25 per method, with matched seed specifications saved on every episode.

## Nominal episode

Separate nominal runs completed successfully for each solver:

| Method | Success | Final target error (m) | Peak cube height (m) |
|---|---:|---:|---:|
| DLS | yes | 0.00264 | 0.56288 |
| Mink | no | 0.04375 | 0.54294 |
| QP | yes | 0.00993 | 0.54958 |

These records are in `results/nominal_final_dls/`, `results/nominal_final_mink/`, and `results/nominal_final_qp/`. The three-method smoke target solve converged for all methods. Its matched short episodes also completed for all three methods; see `results/smoke_final_check/smoke.json`.

## Workspace analysis

The 4 × 4 × 4 grid sampled 64 xyz endpoints per method over x=[0.25, 0.65] m, y=[-0.35, 0.35] m, z=[0.40, 0.75] m. At 20 mm position and 0.15 rad orientation tolerances, each method solved 18/64 endpoints. This is sampled endpoint reachability only. It does not prove exhaustive workspace coverage, collision-free path reachability, or reachability at unsampled targets. The CSV and faceted per-method figure are in `results/workspace_collision_reviewed/`.

## Collision analysis

The nominal DLS audit sampled task-relevant geom distances every ten simulation steps (800 samples over the episode) and separately observed actual MuJoCo contacts. It recorded zero unexpected contact pairs. Minimum sampled signed clearances were 6.87 mm for robot/table, 141.0 mm for robot/floor, and 69.2 mm for arm/cube. The expected hand/cube and cube/table contacts reached -1.47 mm and -2.52 mm signed distance respectively, representing measured penetration during grasp/support contact.

The separate experimental controller gates each of its eight joint-linear waypoint segments using 25 discrete samples and a 2 mm clearance margin. All eight were accepted; the smallest sampled clearance was 6.87 mm. The gated nominal episode succeeded, and its runtime contact monitor found no unexpected contacts. This is a sampled check, not a certified collision-free planner; collisions between samples can be missed. No tray or added obstacle geometry exists. The passive audit and gated path record are under `results/workspace_collision_final/` and `results/collision_aware_gated_corrected/`.

## Challenging case and limitations

The boundary placement case moved the cube and destination to (0.30, -0.28) m and (0.30, 0.28) m. All three methods failed the placement success check, ending at the start location with no lift. The result is saved in `results/challenging_final/boundary_case.json`. This is an observed failure, not an obstacle avoidance trial.

The solvers use a 20 mm position and 0.12 rad orientation convergence threshold, while task success requires final XY error below 10 mm, a lift greater than 45 mm, and final z in (0.39, 0.49) m. The model uses position actuators rather than a torque-level impedance controller. Contact measurements are sampled at a stride of ten simulation steps. The bundled asset checkout includes an Apache License 2.0 text but no repository provenance metadata; preserve the license and resolve attribution/source provenance before public redistribution.

## Conclusion

The bundled Panda model loaded with seven correctly named arm joints, seven arm actuators, a coupled finger pair, and one tendon gripper actuator. Under the 10 mm placement criterion, the saved nominal DLS and QP episodes succeed and Mink fails; matched batch success was 25/25 for DLS, 1/25 for Mink, and 5/25 for QP. DLS achieved substantially smaller mean placement error and lower IK solve time than Mink and QP in these runs. The failures in the boundary case, finite endpoint sampling, and discrete collision checks limit the scope of those conclusions.
