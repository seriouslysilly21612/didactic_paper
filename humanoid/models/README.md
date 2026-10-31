# Models of the humanoid (paper, Sec. VI)

Generated from the URDF of the Unitree G1 with 29 joints published by the manufacturer:
`unitreerobotics/unitree_ros`, commit 5994d4f, `robots/g1_description/g1_29dof.urdf`
(sha256 e1dc8936...e31), BSD-3-Clause (Copyright (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD.,
"Unitree Robotics"). Every number of the original is copied unchanged.

    python3 humanoid/tools/g1_models.py g1_29dof.urdf humanoid/models
    python3 tools/urdf2mjcf.py humanoid/models/g1_fixed.urdf humanoid/models/g1_mujoco.xml --verify

| File | Content |
|---|---|
| `g1_fixed.urdf` | the robot with the pelvis fixed to the world, no geometry: the computed-torque law (RBDL) |
| `g1_mujoco.xml` | the same, for MuJoCo (every number copied; RK4, 1 ms) |
| `g1_left_arm.urdf`, `g1_right_arm.urdf` | the chain of each hand for its IK model: the waist joints fixed at 0, the 7 arm joints, the tool point `<side>_rubber_hand` |
| `g1_joints.txt` | the 29 joints with their limits and effort (the torque limits of the control law) |

`bin/humanoid_sim --mode check` compares the models: the hand positions, M and h over 10^4 random
states, and the IK models of the hands against the full model.
