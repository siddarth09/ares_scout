# 🚀 Missile Control System Simulation

## 📖 Overview
This project simulates a **missile control system** using **state-space modeling, LQR control, and Kalman filtering** to estimate and control the missile's trajectory. It implements a **Longitudinal Autopilot Design** that stabilizes and controls the missile's flight dynamics.

Key features:
- **State-Space Representation** of missile dynamics
- **Linear Quadratic Regulator (LQR)** for control
- **Kalman Filter (LQG)** for state estimation with noisy sensor data
- **Geospatial Computation** for trajectory calculation using the **Haversine Formula**
- **Azimuth and Flight Path Angle Calculation**

This project is based on **"Missile Longitudinal Autopilots: Connections Between Optimal Control and Classical Topologies"** 📄.

---

## 🏗️ Project Structure
The project consists of three core MATLAB scripts:

| File | Description |
|------|------------|
| `main.m` | The main script that initializes parameters, calls functions, and simulates the system. |
| `missile_state_space.m` | Defines the **state-space model** for missile dynamics. |
| `missile_control_system.m` | Implements **LQR control**, system observability analysis, and the closed-loop system design. |

---

## 📂 File Descriptions

### **1️⃣ `main.m`**
This is the **main execution script**. It performs the following steps:
1. **Define Missile Parameters**: Velocity, mass, aerodynamic coefficients, and other physical properties.
2. **Geospatial Calculations**: Uses the **Haversine Formula** to compute the missile's range and trajectory.
3. **Compute State-Space Representation**: Calls `missile_state_space.m` to obtain system matrices.
4. **Apply LQR Control**: Calls `missile_control_system.m` to generate the closed-loop system.
5. **Estimate State Using Kalman Filter**: Implements **Linear Quadratic Gaussian (LQG) control** to handle sensor noise.
6. **Compute Flight Path Parameters**: Calculates azimuth and initial flight path angle.

### **2️⃣ `missile_state_space.m`**
This function **models the missile dynamics** using **state-space representation**:
\[
\dot{x} = Ax + Bu
\]
\[
y = Cx + Du
\]
- Computes **A, B, C, D matrices** based on aerodynamic and physical properties.
- Returns a **state-space system (`sys`)** for further control design.

### **3️⃣ `missile_control_system.m`**
This function **designs an LQR controller** for missile stability:
1. Computes the **controllability** and **observability** matrices.
2. Solves the **LQR problem**:
   \[
   K = \text{lqr}(A, B, Q, R)
   \]
3. Forms the **closed-loop system**:
   \[
   A_{\text{cl}} = A - B K
   \]
4. Returns the **controlled system (`sys_cl`)**.

---

## 📊 How to Run the Code
1. **Open MATLAB**.
2. **Run `main.m`**:
   ```matlab
   main


### REFERENCES (additional)
Comparative Study of Optimal Multivariable LQR and MPC Controllers for Unmanned Combat Air Systems in Trajectory Tracking
https://www.ias.ac.in/article/fulltext/sadh/047/0038?utm_source=chatgpt.com
