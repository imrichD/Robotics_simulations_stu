# Robotics_simulations_stu

Robotics simulations and visualization projects (STU FEI), implemented in
**Python** and **MATLAB/Simulink**.

This repository contains three course assignments:
1) forward kinematics with homogeneous transformations,
2) dynamics + position control of a 2-DOF manipulator,
3) kinematics + visualization of a differential drive robot.

---

## Projects

### 1) Aerial ladder platform — Forward kinematics (Python)
**Folder:** `assignment_01_aerial_ladder_fk_python/`  
**Report:** `assignment_01_aerial_ladder_fk_python/zad1-Dzvoňár.pdf`  
**Code:** `assignment_01_aerial_ladder_fk_python/DzvonarRobZadanie1.py`

**What it does**
- Builds a chain of homogeneous transformations \(T_{0k}\)
- Computes the end-point position \(p_0 = T_{0k} p_k\)
- Visualizes the simplified mechanism + coordinate frames
- Computes/plots workspace envelopes in **XY** and **XZ** planes

**Run**
```bash
python assignment_01_aerial_ladder_fk_python/DzvonarRobZadanie1.py
```

Dependencies: `numpy`, `matplotlib`.

---

### 2) 2-DOF manipulator — Dynamics + PID position control (MATLAB/Simulink)
**Folder:** `assignment_02_2dof_manipulator_dynamics_simulink/`  
**Report:** `assignment_02_2dof_manipulator_dynamics_simulink/zad2-Dzvoňár.pdf`  
**Simulink model:** `assignment_02_2dof_manipulator_dynamics_simulink/ROB2_schema_Dzvonar.slx`  
**MATLAB init script:** `assignment_02_2dof_manipulator_dynamics_simulink/Script_zad2_Dzvonar.m`

**What it does**
- Implements a dynamic model derived with Lagrange equations:
  \[
  M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau
  \]
- Verifies open-loop behavior (angles, velocities)
- Designs joint-level **PID** controllers for both axes
- Experiments with **torque saturation** (actuator limits)
- Plots joint angles and motor torques

**Run**
1. Open MATLAB in the repo root
2. Run the init script:
   - `Script_zad2_Dzvonar.m`
3. Open the Simulink model:
   - `ROB2_schema_Dzvonar.slx`
4. Run the simulation

---

### 3) Differential drive robot — Kinematics + visualization (MATLAB)
**Folder:** `assignment_03_differential_drive_robot_matlab/`  
**Report:** `assignment_03_differential_drive_robot_matlab/zad3-Dzvonar.pdf`  
**Code:** `Rob3U1.m`, `Rob3U2.m`, `Rob3U3.m`, `Rob3U4.m`

**What it does**
- Simulates a differential drive robot with:
  - wheel track \(L = 200\ \text{mm}\)
  - wheel radius \(r = 50\ \text{mm}\)
- Uses standard kinematics:
  \[
  v_T = \frac{v_R + v_L}{2}, \quad \omega_T = \frac{v_R - v_L}{L}
  \]
- Generates and visualizes:
  - trajectories of robot center + left/right wheel tracks
  - time plots of wheel/center velocities
  - square trajectory (user-defined side length)
  - curve trajectory (arc–line–arc)
  - interactive control (WASD/arrows) with logged results

**Run**
Open MATLAB in:
`assignment_03_differential_drive_robot_matlab/`

Run the scripts (order depends on your implementation):
- `Rob3U1.m` (task 1)
- `Rob3U2.m` (task 2)
- `Rob3U3.m` (task 3)
- `Rob3U4.m` (task 4)

---

## Notes
- Recommended unit convention: meters, seconds, radians.
- Generated artifacts (Simulink cache, Python bytecode, etc.) are excluded via
  `.gitignore` to keep the repository clean.

## Author
Imrich Dzvoňár (STU FEI), 2025
