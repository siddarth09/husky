# 🐾 Quadruped Robot MPC Control Project

## 📋 Project Overview

This project implements a **ROS 2-based simulation and control stack** for a custom quadruped robot using **Model Predictive Control (MPC)**. It includes full-body URDF modeling, joint-level control via `ros2_control`, inverse kinematics, gait planning (trot), and CasADi-based MPC for stable, optimized locomotion. The robot is simulated in **Gazebo Harmonic** with Bullet + Featherstone physics.

---

## ✅ Gantt-Style Project Checklist

| ✅  | Week | Phase                  | Task                                             | Duration | Depends On          |
| -- | ---- | ---------------------- | ------------------------------------------------ | -------- | ------------------- |
| ✅  | 1    | Simulation Setup       | Finalize URDF and Add `<ros2_control>` tags      | 3 days   | —                   |
| ✅  | 1    | Simulation Setup       | Setup Gazebo Harmonic launch + plugin config     | 2 days   | URDF complete       |
| ✅  | 2    | Controller Integration | Create controllers.yaml for joint control        | 1 day    | ros2\_control ready |
| ✅  | 2    | Controller Integration | Load robot into Gazebo with controllers          | 1 day    | Launch setup ready  |
| ⬜️  | 2-3  | Kinematics             | Define leg kinematics (link lengths, layout)     | 2 days   | URDF ready          |
| ⬜️  | 3    | Kinematics             | Implement inverse kinematics (analytical/solver) | 2 days   | Kinematics defined  |
| ⬜️ | 4    | Gait Planning          | Implement basic foot trajectory (trot gait)      | 3 days   | IK complete         |
| ⬜️ | 4    | Gait Planning          | Add timing + phasing logic to gait planner       | 1 day    | Gait planner ready  |
| ⬜️ | 5    | Dynamics & MPC         | Build simplified dynamics model (CoM, legs)      | 2 days   | Kinematics ready    |
| ⬜️ | 5    | MPC Core               | Setup MPC using CasADi/acados                    | 3 days   | Dynamics defined    |
| ⬜️ | 6    | Integration            | Connect MPC to ROS 2 + IK → joint commands       | 2 days   | MPC, IK complete    |
| ⬜️ | 6-7  | Testing & Tuning       | Tune Q/R weights, gains, gait params             | 1 week   | System integrated   |
| ⬜️ | 8+   | Advanced (Optional)    | Terrain testing, sensors, adaptivity             | ongoing  | MVP system working  |

---

## 📁 Folder Structure

```
husky/
├── husky_description/         # URDF, meshes, xacro, ros2_control tags
├── husky_controllers/         # Controller YAMLs, IK scripts, gait planner
└── README.md                  # This file
```

---

##  Getting Started

```bash
cd husky_ws/
colcon build
source install/setup.bash
ros2 launch husky_description spawn_robot.launch.py
```

---

##  Next Steps

* [ ] Implement gait planner
* [ ] Integrate MPC node
* [ ] Tune for terrain adaptivity

---

## 📈 Future Enhancements

* [ ] Add joystick/teleop gait switching
* [ ] Visualize planned vs. actual foot trajectories
* [ ] Implement state estimation (EKF or base odometry)
* [ ] Extend MPC for push recovery and adaptive gaiting
* [ ] Simulate rough terrain and contact force logging
* [ ] Publish a full demo video and GitHub documentation

---

## 🧪 Testing Checklist

* [ ] Robot stands stably in Gazebo Harmonic
* [ ] All joints receive valid position commands
* [ ] Contact sensors trigger properly on ground contact
* [ ] IK returns expected joint configurations per leg
* [ ] MPC outputs consistent foot targets at runtime


