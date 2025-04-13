# Missile Autopilot Controller Comparison

This repository presents a comprehensive comparison of three control strategies used in missile autopilot design:

- **State Feedback with Integral Action (SFI)**
- **Linear Quadratic Regulator (LQR + Integral Augmentation)**
- **Model Predictive Control (MPC)**

Each controller is implemented on a realistic linearized missile model and evaluated for guidance performance, control efficiency, and trajectory tracking.

---

## 1. Introduction
Missile Guidance, Navigation, and Control (GNC) systems are crucial for accurate targeting and stability. In modern applications, we must evaluate not only tracking performance but also robustness, energy efficiency, and constraint handling. This project implements and visualizes three prominent control strategies in MATLAB, assessing each for real-time missile trajectory execution.

---

## 2. State Feedback with Integral Action (SFI)

### a) Controller Algorithm (Pseudocode Outline)
```matlab
Initialize state x = [alpha; q];
Initialize integral error int_error = 0;
Set initial position, altitude, and pitch angle;

for each time step:
    Calculate Line-of-Sight (LOS) angle;
    Compute LOS error = LOS_angle - current_pitch;
    Compute desired acceleration Az_ref = -K_guidance * LOS_error;
    Add launch boost (if in early phase);
    Measure actual Az = C * x;
    Update integral error: int_error += (Az_ref - Az) * dt;
    Compute control input: u = -K * x - Ki * int_error;
    Update state using Euler integration;
    Update position using velocity projection;
end
```

### b) Figures
- **State Trajectories and Control Input:**
 
 ![Image](https://github.com/user-attachments/assets/d4e84cc1-dc86-4635-83a4-073b5c5acb0e)
  

- **Tracking Error ($A_z$, $q$):**
  
  ![Image](https://github.com/user-attachments/assets/537a778d-df36-4d6e-bb32-9b5c75a09e43)

> SFI maintains stable pitch rate and angle of attack with minimal control effort, though it struggles to tightly track the $A_z$ reference.

---

## 3. Linear Quadratic Regulator (LQR + Integral)

### a) Controller Algorithm (Pseudocode Outline)
```matlab
Discretize missile system to obtain Ad, Bd, Cd;
Augment with integral state to track Az error;
Solve DARE using dlqr to get gain K_aug = [K, Ki];

for each time step:
    Compute Az error = Az_ref - Az;
    Update integral error: int_e += error * Ts;
    Form augmented state: x_aug = [x; int_e];
    Compute control input: u = -K_aug * x_aug;
    Propagate next state and update trajectory;
end
```

### b) Figures
- **State Trajectories and Control Input:**
  
 ![Image](https://github.com/user-attachments/assets/9ca938bc-c3cc-4f71-ba94-7a7ee64b7fea)
- **Tracking Error ($A_z$, $q$):**
  
  ![Image](https://github.com/user-attachments/assets/a0a3f1a0-1f59-450e-9593-4b3ca63296db)
  
> LQR with integral action shows smooth convergence and near-perfect tracking of both acceleration and pitch rate.

---

## 4. Model Predictive Control (MPC)

### a) Controller Algorithm (Pseudocode Outline)
```matlab
Discretize model: Ad, Bd, Cd;
Define prediction horizon N and weights Qy, R, Qterm;
Symbolically define future states X and inputs U;
for each time step:
    Construct cost function (tracking + effort + terminal);
    Enforce dynamics as equality constraints;
    Add input bounds (e.g. u in [-0.3, 0.2]);
    Solve NLP using CasADi + IPOPT;
    Apply first optimal input u = U(1);
    Propagate state forward and update trajectory;
end
```

### b) Figures
- **State Trajectories and Control Input:**
  
  ![Image](https://github.com/user-attachments/assets/17eae54d-4396-4baf-b54b-012c7043ef1a)
- **Tracking Error ($A_z$, $q$):**
  
  ![Image](https://github.com/user-attachments/assets/831126bf-88ff-4050-acc2-c899891a624f)

> MPC offers aggressive and efficient control with fast convergence. It slightly underperforms in final Az tracking due to soft constraints.

---

## 5. Performance Evaluation

| Metric                              | SFI    | LQR + Integral | MPC (Ours) | Unit   |
|-------------------------------------|--------|----------------|------------|--------|
| Time to Reach Target Altitude       | 13.80  | 10.00          | **8.00**   | s      |
| Avg Control Input Magnitude         | **0.04** | 0.10           | 0.08       | rad    |
| Max Control Input                   | **0.11** | 0.13           | 0.30       | rad    |
| Final Az Tracking Error             | 19.91  | **0.00**       | 14.97      | g      |
| Final Pitch Rate Error (q)          | **0.04** | 0.16           | **0.00**   | rad/s  |
| Total Control Effort (u)             | 48.94  | 10.34          | **6.06**   | rad    |

### Explanation:
- **Time to Reach Target:** Indicates responsiveness. Lower is better.
- **Control Input Metrics:** Show actuator workload.
- **Tracking Errors:** Assess steady-state and dynamic accuracy.
- **Total Control Effort:** Measures energy consumed in maneuvering.

---

## 6. Demo Video


https://github.com/user-attachments/assets/03c58922-3aa7-4e7f-9103-baea0c317722

---

## Authors
- [Siddarth Dayasagar](https://github.com/siddarth09)
- [Sarthak Talwadkar](https://github.com/sarthak-talwadkar)
- [Ket Bhikadiya]( https://github.com/bhikadiya-k)
- [Chaitanya Salve](https://github.com/salve-c)

Feel free to fork, star, or contribute to this repository.

