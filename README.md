

# **Missile Guidance using Model Predictive Control (MPC)**

## **Overview**

This project implements a **Model Predictive Control (MPC)** scheme to guide a missile from an initial altitude towards a specified 3D target location, while accounting for pitch dynamics, acceleration tracking, and trajectory prediction. The control is realized using **CasADi** for nonlinear optimization and **MATLAB** for simulation and visualization.

https://github.com/user-attachments/assets/56963eb2-63f4-478c-9689-39140d228990


---

## **1. Objective**

Design a guidance controller that:
- Accurately tracks desired missile dynamics (normal acceleration and pitch rate),
- Predicts and controls future behavior,
- Guides the missile to a 3D target (in x-z or x-y-z space),
- Animates the missile's trajectory and body.

---

## **2. Missile Dynamics Model**

The longitudinal dynamics of the missile are modeled as a discrete-time state-space system:

**States:**
- `x = [alpha; q]` (angle of attack and pitch rate)

**Input:**
- `u` = fin deflection

**Output:**
- `y = [az; q]` (normal acceleration and pitch rate)

### State-space matrices (continuous):
```matlab
A = [-1.064 1.000; 290.26 0.00];
B = [-0.25; -331.40];
C = [-123.34 0.00; 0.00 1.00];
D = [0;0];
```

Discretization using sampling time `Ts = 0.1`:
```matlab
sysd = c2d(ss(A,B,C,D), Ts);
```

---

## **3. Model Predictive Control (MPC)**

MPC is a receding-horizon control strategy that:
1. Predicts system behavior over a future window (`N` steps),
2. Solves an optimization problem to minimize a cost,
3. Applies only the first control input,
4. Repeats this process at each step.

### Cost Function Components:
- Output tracking: `(y_k - y_ref_k)' * Qy * (y_k - y_ref_k)`
- Control effort: `u_k' * R * u_k`
- Terminal target error (soft constraint): `(final_pos - target)' * Q_terminal * (final_pos - target)`

### CasADi MPC Implementation:

**Dynamics Function:**
```matlab
x_next = Ad*x + Bd*u;
f = Function('f',{x,u},{x_next});
```

**Optimization Variables:**
```matlab
U = MX.sym('U', nu, N);     % Control inputs
X = MX.sym('X', nx, N+1);   % States
```

**Constraints:**
```matlab
g{end+1} = X(:,1) - x0;                   % Initial condition
g{end+1} = X(:,k+1) - f(X(:,k), U(:,k));  % System dynamics
```

**Tracking Cost:**
```matlab
cost = cost + (y_k - y_ref_k)' * Qy * (y_k - y_ref_k) + u_k' * R * u_k;
```

**Terminal Soft Constraint:**
```matlab
cost = cost + (final_pos - target)' * Q_terminal * (final_pos - target);
```

---

## **4. Trajectory Estimation**

Using estimated pitch and constant velocity, future positions are calculated:

```matlab
pitch_angle(t+1) = pitch_angle(t) + q * Ts;

x_pos(t+1) = x_pos(t) + Vel * cos(pitch_angle(t+1)) * Ts;
z_pos(t+1) = z_pos(t) + Vel * sin(pitch_angle(t+1)) * Ts;
```

**For 3D extension (with yaw):**
```matlab
x_vel = Vel * cos(pitch) * cos(yaw);
y_vel = Vel * cos(pitch) * sin(yaw);
z_vel = Vel * sin(pitch);
```

---

## **5. Target & Soft Constraints**

The missile is softly guided to a target position, e.g.:
```matlab
target = [10500; 300];  % Target x, z in meters
```

Soft constraint added to the cost, allowing flexibility:
```matlab
cost = cost + (final_pos - target)' * Q_terminal * (final_pos - target);
```

---

## **6. Visualization and Animation**

### Static 3D Plot:
```matlab
plot3(x_pos/1000, y_pos/1000, z_pos/1000);  % In kilometers
```

### Animated Missile Body (Red Cone):
```matlab
R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
```
This rotation matrix aligns the cone with the missile’s current pitch.

Animation is updated inside a loop using:
```matlab
set(missile_surf, 'XData', Xr, 'YData', Yr, 'ZData', Zr);
```

---

## **7. Why Use MPC for Missile Guidance?**

- ✅ **Predictive**: Plans ahead based on future behavior.
- ✅ **Constraint-aware**: Can limit acceleration, pitch, deflection, etc.
- ✅ **Optimal**: Finds the best control inputs at each step.
- ✅ **Multi-objective**: Track a dynamic reference AND reach a goal.
- ✅ **Flexible**: Extendable to full 6DOF, wind effects, obstacles.

---

## **8. Future Extensions**

- Add full lateral motion (turning dynamics)
- Obstacle avoidance with constraint zones
- Nonlinear missile dynamics modeling
- Time-optimal or energy-optimal descent
- Real-time implementation on embedded hardware

