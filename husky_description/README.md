# 🗓️ Gantt-Style Project Breakdown for Quadruped Robot with MPC

| Week | Phase                      | Task                                               | Duration   | Depends On            |
|------|----------------------------|----------------------------------------------------|------------|------------------------|
| 1    | Simulation Setup           | ✅ Finalize URDF and Add `<ros2_control>` tags     | 3 days     | —                      |
| 1    | Simulation Setup           | 🔧 Setup Gazebo Harmonic launch + plugin config    | 2 days     | URDF complete          |
| 2    | Controller Integration     | ⚙️  Create controllers.yaml for joint control       | 1 day      | ros2_control ready     |
| 2    | Controller Integration     | 🚀 Load robot into Gazebo with controllers         | 1 day      | Launch setup ready     |
| 2-3  | Kinematics                 | 🧠 Define leg kinematics (link lengths, layout)     | 2 days     | URDF ready             |
| 3    | Kinematics                 | 📐 Implement inverse kinematics (analytical/solver)| 2 days     | Kinematics defined      |
| 4    | Gait Planning              | 🐾 Implement basic foot trajectory (trot gait)     | 3 days     | IK complete            |
| 4    | Gait Planning              | ⏱ Add timing + phasing logic to gait planner       | 1 day      | Gait planner ready     |
| 5    | Dynamics & MPC             | 🧮 Build simplified dynamics model (CoM, legs)      | 2 days     | Kinematics ready       |
| 5    | MPC Core                   | 🧠 Setup MPC using CasADi/acados                   | 3 days     | Dynamics defined        |
| 6    | Integration                | 🔁 Connect MPC to ROS 2 + IK → joint commands       | 2 days     | MPC, IK complete       |
| 6-7  | Testing & Tuning           | 🧪 Tune Q/R weights, gains, gait params             | 1 week     | System integrated      |
| 8+   | Advanced (Optional)        | 🪨 Terrain testing, sensors, adaptivity             | ongoing    | MVP system working     |
