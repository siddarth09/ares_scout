
```matlab
addpath('C:\Users\Siddarth\Documents\MATLAB')
import casadi.*
```
# Missile Longitudinal Dynamics
```matlab
A = [-1.064 1.000; 290.26 0.00];
B = [-0.25; -331.40];
C = [-123.34 0.00; 0.00 1.00];
D = [0;0];
plant = ss(A,B,C,D);

% Check system properties
if rank(ctrb(A,B)) == size(A,1), disp("System is controllable"), end
```

```matlabTextOutput
System is controllable
```

```matlab
if rank(obsv(A,C)) == size(A,1), disp("System is observable"), end
```

```matlabTextOutput
System is observable
```
# Discretization
```matlab
Ts = 0.1;
sysd = c2d(plant,Ts);
Ad = sysd.A; Bd = sysd.B; Cd = sysd.C; Dd = sysd.D;

nx = size(Ad,1); nu = size(Bd,2); ny = size(Cd,1);
Qy = diag([2, 0.5]); R = 1;
```
# CasADi symbolic model
```matlab
x = MX.sym('x',nx); u = MX.sym('u',nu);
x_next = Ad*x + Bd*u;
f = Function('f',{x,u},{x_next});
```
# MPC setup
```matlab
N = 10;
x0_val = [0; 0];
U = MX.sym('U',nu,N);
X = MX.sym('X',nx,N+1);
x0 = MX.sym('x0',nx);
```
# Reference tracking
```matlab
Az_step = linspace(0, 15, N);  % target normal acceleration (g)
q_ref = zeros(1, N);           % target pitch rate
y_ref = [Az_step; q_ref];
```
# 3D estimation parameters
```matlab
Vel = 1021.08;  % m/s constant forward velocity
n_steps = 50;
x_sim = zeros(nx, n_steps+1); u_sim = zeros(nu, n_steps);
x_sim(:,1) = x0_val;
pitch_angle = zeros(1,n_steps+1);
x_pos = zeros(1,n_steps+1);
z_pos = zeros(1,n_steps+1);

x_pos(1)=1590;
z_pos(1) = 10000;  % initial height
```
# MPC loop
```matlab
for t = 1:n_steps
    cost = 0; g = {};
    g{end+1} = X(:,1) - x0;

    % Predict pitch angle symbolically
    pitch_pred = MX.zeros(1,N+1);
    pitch_pred(1) = pitch_angle(t);     

    % Predict 3D trajectory symbolically
    x_pos_pred = MX.zeros(1,N+1);
    z_pos_pred = MX.zeros(1,N+1);
    x_pos_pred(1) = x_pos(t);
    z_pos_pred(1) = z_pos(t);

    for k = 1:N
        pitch_pred(k+1) = pitch_pred(k) + X(2,k)*Ts;
        x_k = X(:,k); u_k = U(:,k);
        x_next = f(x_k, u_k);
        g{end+1} = X(:,k+1) - x_next;

        % Tracking cost
        y_k = C * x_k;
        y_ref_k = y_ref(:,k);
        cost = cost+(y_k - y_ref_k)' * Qy * (y_k - y_ref_k) + u_k'*R*u_k;

        % Position integration
        x_pos_pred(k+1) = x_pos_pred(k) + Vel*cos(pitch_pred(k+1))*Ts;
        z_pos_pred(k+1) = z_pos_pred(k) + Vel*sin(pitch_pred(k+1))*Ts;
    end

    % Terminal soft constraint to target position
    target = [10500; 300];
    final_pos = [x_pos_pred(end); z_pos_pred(end)];
    Qterm = 0.01 * eye(2);
    cost = cost + (final_pos - target)' * Qterm * (final_pos - target);

    % Solve
    opt_vars = [reshape(X, nx*(N+1), 1); reshape(U, nu*N, 1)];
    nlp = struct('x',opt_vars,'f',cost,'g',vertcat(g{:}),'p',x0);
    solver = nlpsol('solver','ipopt',nlp,struct('ipopt',struct('print_level',0)));

    % Initial guesses
    U0 = zeros(nu,N); X0 = repmat(x_sim(:,t),1,N+1); w0 = [X0(:);U0(:)];
    sol = solver('x0',w0,'p',x_sim(:,t),'lbg',0,'ubg',0);
    w_opt = full(sol.x);
    U_opt = reshape(w_opt(nx*(N+1)+1:end),nu,N);

    % Apply control and update states
    u0 = U_opt(:,1); u_sim(:,t) = u0;
    x_sim(:,t+1) = Ad * x_sim(:,t) + Bd * u0;

    % Update pitch and trajectory
    pitch_angle(t+1) = pitch_angle(t) + x_sim(2,t)*Ts;
    x_pos(t+1) = x_pos(t) + Vel*cos(pitch_angle(t+1))*Ts;
    z_pos(t+1) = z_pos(t) + Vel*sin(pitch_angle(t+1))*Ts;
end
```

```matlabTextOutput
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0) 119.00us ( 23.80us)         5
       nlp_g  |        0 (       0)  63.00us ( 12.60us)         5
  nlp_grad_f  |        0 (       0) 205.00us ( 34.17us)         6
  nlp_hess_l  |   1.00ms (250.00us)   1.15ms (287.50us)         4
   nlp_jac_g  |        0 (       0) 167.00us ( 27.83us)         6
       total  |   9.00ms (  9.00ms)   9.39ms (  9.39ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0) 366.00us ( 14.08us)        26
       nlp_g  |   1.00ms ( 38.46us) 222.00us (  8.54us)        26
  nlp_grad_f  |   1.00ms ( 45.45us) 575.00us ( 26.14us)        22
  nlp_hess_l  |   3.00ms (150.00us)   3.71ms (185.65us)        20
   nlp_jac_g  |        0 (       0) 530.00us ( 24.09us)        22
       total  |  32.00ms ( 32.00ms)  31.53ms ( 31.53ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  96.00us ( 12.00us)         8
       nlp_g  |        0 (       0)  60.00us (  7.50us)         8
  nlp_grad_f  |        0 (       0) 244.00us ( 27.11us)         9
  nlp_hess_l  |        0 (       0)   1.24ms (177.00us)         7
   nlp_jac_g  |        0 (       0) 227.00us ( 25.22us)         9
       total  |  11.00ms ( 11.00ms)  10.74ms ( 10.74ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |   1.00ms (200.00us)  83.00us ( 16.60us)         5
       nlp_g  |        0 (       0)  50.00us ( 10.00us)         5
  nlp_grad_f  |   1.00ms (166.67us) 162.00us ( 27.00us)         6
  nlp_hess_l  |   1.00ms (250.00us) 745.00us (186.25us)         4
   nlp_jac_g  |        0 (       0) 139.00us ( 23.17us)         6
       total  |   7.00ms (  7.00ms)   6.81ms (  6.81ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  63.00us ( 15.75us)         4
       nlp_g  |        0 (       0)  39.00us (  9.75us)         4
  nlp_grad_f  |        0 (       0) 140.00us ( 28.00us)         5
  nlp_hess_l  |   1.00ms (333.33us) 703.00us (234.33us)         3
   nlp_jac_g  |        0 (       0) 124.00us ( 24.80us)         5
       total  |   6.00ms (  6.00ms)   6.02ms (  6.02ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  80.00us ( 20.00us)         4
       nlp_g  |        0 (       0)  60.00us ( 15.00us)         4
  nlp_grad_f  |        0 (       0) 151.00us ( 30.20us)         5
  nlp_hess_l  |   1.00ms (333.33us) 672.00us (224.00us)         3
   nlp_jac_g  |        0 (       0) 121.00us ( 24.20us)         5
       total  |   6.00ms (  6.00ms)   6.39ms (  6.39ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  50.00us ( 12.50us)         4
       nlp_g  |        0 (       0)  31.00us (  7.75us)         4
  nlp_grad_f  |        0 (       0) 131.00us ( 26.20us)         5
  nlp_hess_l  |        0 (       0) 470.00us (156.67us)         3
   nlp_jac_g  |        0 (       0) 110.00us ( 22.00us)         5
       total  |   5.00ms (  5.00ms)   5.14ms (  5.14ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  88.00us ( 22.00us)         4
       nlp_g  |        0 (       0)  63.00us ( 15.75us)         4
  nlp_grad_f  |        0 (       0) 147.00us ( 29.40us)         5
  nlp_hess_l  |   2.00ms (666.67us) 771.00us (257.00us)         3
   nlp_jac_g  |        0 (       0) 145.00us ( 29.00us)         5
       total  |   7.00ms (  7.00ms)   6.73ms (  6.73ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  83.00us ( 20.75us)         4
       nlp_g  |        0 (       0)  52.00us ( 13.00us)         4
  nlp_grad_f  |        0 (       0) 189.00us ( 37.80us)         5
  nlp_hess_l  |   1.00ms (333.33us) 723.00us (241.00us)         3
   nlp_jac_g  |        0 (       0) 123.00us ( 24.60us)         5
       total  |   5.00ms (  5.00ms)   6.11ms (  6.11ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  59.00us ( 14.75us)         4
       nlp_g  |        0 (       0)  35.00us (  8.75us)         4
  nlp_grad_f  |        0 (       0) 139.00us ( 27.80us)         5
  nlp_hess_l  |        0 (       0) 572.00us (190.67us)         3
   nlp_jac_g  |   1.00ms (200.00us) 118.00us ( 23.60us)         5
       total  |   6.00ms (  6.00ms)   5.77ms (  5.77ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  68.00us ( 17.00us)         4
       nlp_g  |        0 (       0)  45.00us ( 11.25us)         4
  nlp_grad_f  |        0 (       0) 158.00us ( 31.60us)         5
  nlp_hess_l  |   1.00ms (333.33us) 647.00us (215.67us)         3
   nlp_jac_g  |        0 (       0) 129.00us ( 25.80us)         5
       total  |   7.00ms (  7.00ms)   6.24ms (  6.24ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  73.00us ( 18.25us)         4
       nlp_g  |        0 (       0)  48.00us ( 12.00us)         4
  nlp_grad_f  |        0 (       0) 173.00us ( 34.60us)         5
  nlp_hess_l  |   1.00ms (333.33us) 724.00us (241.33us)         3
   nlp_jac_g  |        0 (       0) 138.00us ( 27.60us)         5
       total  |   6.00ms (  6.00ms)   6.31ms (  6.31ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  85.00us ( 21.25us)         4
       nlp_g  |        0 (       0)  50.00us ( 12.50us)         4
  nlp_grad_f  |   1.00ms (200.00us) 188.00us ( 37.60us)         5
  nlp_hess_l  |   1.00ms (333.33us) 889.00us (296.33us)         3
   nlp_jac_g  |        0 (       0) 137.00us ( 27.40us)         5
       total  |   7.00ms (  7.00ms)   6.43ms (  6.43ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  80.00us ( 20.00us)         4
       nlp_g  |        0 (       0)  45.00us ( 11.25us)         4
  nlp_grad_f  |   1.00ms (200.00us) 169.00us ( 33.80us)         5
  nlp_hess_l  |   1.00ms (333.33us) 746.00us (248.67us)         3
   nlp_jac_g  |        0 (       0) 133.00us ( 26.60us)         5
       total  |   7.00ms (  7.00ms)   6.52ms (  6.52ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0) 116.00us ( 29.00us)         4
       nlp_g  |   1.00ms (250.00us)  52.00us ( 13.00us)         4
  nlp_grad_f  |        0 (       0) 174.00us ( 34.80us)         5
  nlp_hess_l  |   1.00ms (333.33us) 581.00us (193.67us)         3
   nlp_jac_g  |   1.00ms (200.00us) 128.00us ( 25.60us)         5
       total  |   6.00ms (  6.00ms)   6.10ms (  6.10ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |   1.00ms (250.00us)  80.00us ( 20.00us)         4
       nlp_g  |        0 (       0)  33.00us (  8.25us)         4
  nlp_grad_f  |   1.00ms (200.00us) 140.00us ( 28.00us)         5
  nlp_hess_l  |   2.00ms (666.67us) 503.00us (167.67us)         3
   nlp_jac_g  |   1.00ms (200.00us) 132.00us ( 26.40us)         5
       total  |   6.00ms (  6.00ms)   5.51ms (  5.51ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  54.00us ( 13.50us)         4
       nlp_g  |        0 (       0)  34.00us (  8.50us)         4
  nlp_grad_f  |        0 (       0) 139.00us ( 27.80us)         5
  nlp_hess_l  |        0 (       0) 527.00us (175.67us)         3
   nlp_jac_g  |        0 (       0) 109.00us ( 21.80us)         5
       total  |   5.00ms (  5.00ms)   5.47ms (  5.47ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  83.00us ( 20.75us)         4
       nlp_g  |        0 (       0)  46.00us ( 11.50us)         4
  nlp_grad_f  |        0 (       0) 135.00us ( 27.00us)         5
  nlp_hess_l  |        0 (       0) 724.00us (241.33us)         3
   nlp_jac_g  |   1.00ms (200.00us) 116.00us ( 23.20us)         5
       total  |   6.00ms (  6.00ms)   6.27ms (  6.27ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  61.00us ( 15.25us)         4
       nlp_g  |        0 (       0)  33.00us (  8.25us)         4
  nlp_grad_f  |        0 (       0) 166.00us ( 33.20us)         5
  nlp_hess_l  |        0 (       0) 601.00us (200.33us)         3
   nlp_jac_g  |        0 (       0) 142.00us ( 28.40us)         5
       total  |   6.00ms (  6.00ms)   6.04ms (  6.04ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  48.00us ( 12.00us)         4
       nlp_g  |        0 (       0)  29.00us (  7.25us)         4
  nlp_grad_f  |        0 (       0) 120.00us ( 24.00us)         5
  nlp_hess_l  |        0 (       0) 498.00us (166.00us)         3
   nlp_jac_g  |        0 (       0) 111.00us ( 22.20us)         5
       total  |   5.00ms (  5.00ms)   5.23ms (  5.23ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  50.00us ( 12.50us)         4
       nlp_g  |        0 (       0)  30.00us (  7.50us)         4
  nlp_grad_f  |        0 (       0) 128.00us ( 25.60us)         5
  nlp_hess_l  |   1.00ms (333.33us) 541.00us (180.33us)         3
   nlp_jac_g  |   1.00ms (200.00us) 111.00us ( 22.20us)         5
       total  |   6.00ms (  6.00ms)   5.28ms (  5.28ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  55.00us ( 13.75us)         4
       nlp_g  |        0 (       0)  31.00us (  7.75us)         4
  nlp_grad_f  |        0 (       0) 133.00us ( 26.60us)         5
  nlp_hess_l  |        0 (       0) 586.00us (195.33us)         3
   nlp_jac_g  |        0 (       0) 111.00us ( 22.20us)         5
       total  |   5.00ms (  5.00ms)   5.39ms (  5.39ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  52.00us ( 13.00us)         4
       nlp_g  |        0 (       0)  31.00us (  7.75us)         4
  nlp_grad_f  |        0 (       0) 132.00us ( 26.40us)         5
  nlp_hess_l  |   1.00ms (333.33us) 536.00us (178.67us)         3
   nlp_jac_g  |        0 (       0) 105.00us ( 21.00us)         5
       total  |   4.00ms (  4.00ms)   5.27ms (  5.27ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  63.00us ( 15.75us)         4
       nlp_g  |        0 (       0)  36.00us (  9.00us)         4
  nlp_grad_f  |        0 (       0) 145.00us ( 29.00us)         5
  nlp_hess_l  |        0 (       0) 668.00us (222.67us)         3
   nlp_jac_g  |        0 (       0) 128.00us ( 25.60us)         5
       total  |   6.00ms (  6.00ms)   5.74ms (  5.74ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0) 102.00us ( 34.00us)         3
       nlp_g  |   1.00ms (333.33us)  42.00us ( 14.00us)         3
  nlp_grad_f  |        0 (       0) 135.00us ( 33.75us)         4
  nlp_hess_l  |        0 (       0) 733.00us (366.50us)         2
   nlp_jac_g  |        0 (       0) 106.00us ( 26.50us)         4
       total  |   5.00ms (  5.00ms)   5.32ms (  5.32ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  64.00us ( 21.33us)         3
       nlp_g  |        0 (       0)  34.00us ( 11.33us)         3
  nlp_grad_f  |        0 (       0) 156.00us ( 39.00us)         4
  nlp_hess_l  |   1.00ms (500.00us) 652.00us (326.00us)         2
   nlp_jac_g  |        0 (       0) 114.00us ( 28.50us)         4
       total  |   5.00ms (  5.00ms)   5.32ms (  5.32ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  37.00us ( 12.33us)         3
       nlp_g  |        0 (       0)  24.00us (  8.00us)         3
  nlp_grad_f  |        0 (       0) 108.00us ( 27.00us)         4
  nlp_hess_l  |        0 (       0) 354.00us (177.00us)         2
   nlp_jac_g  |        0 (       0)  90.00us ( 22.50us)         4
       total  |   4.00ms (  4.00ms)   4.25ms (  4.25ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  74.00us ( 24.67us)         3
       nlp_g  |        0 (       0)  45.00us ( 15.00us)         3
  nlp_grad_f  |        0 (       0) 142.00us ( 35.50us)         4
  nlp_hess_l  |   1.00ms (500.00us) 527.00us (263.50us)         2
   nlp_jac_g  |        0 (       0) 112.00us ( 28.00us)         4
       total  |   6.00ms (  6.00ms)   5.52ms (  5.52ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  81.00us ( 27.00us)         3
       nlp_g  |        0 (       0)  60.00us ( 20.00us)         3
  nlp_grad_f  |        0 (       0) 153.00us ( 38.25us)         4
  nlp_hess_l  |        0 (       0) 593.00us (296.50us)         2
   nlp_jac_g  |        0 (       0) 115.00us ( 28.75us)         4
       total  |  43.00ms ( 43.00ms)  43.31ms ( 43.31ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  97.00us ( 32.33us)         3
       nlp_g  |        0 (       0)  57.00us ( 19.00us)         3
  nlp_grad_f  |        0 (       0) 221.00us ( 55.25us)         4
  nlp_hess_l  |        0 (       0) 873.00us (436.50us)         2
   nlp_jac_g  |        0 (       0) 189.00us ( 47.25us)         4
       total  |   8.00ms (  8.00ms)   8.04ms (  8.04ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0) 111.00us ( 37.00us)         3
       nlp_g  |        0 (       0)  52.00us ( 17.33us)         3
  nlp_grad_f  |        0 (       0) 164.00us ( 41.00us)         4
  nlp_hess_l  |   1.00ms (500.00us) 852.00us (426.00us)         2
   nlp_jac_g  |        0 (       0) 133.00us ( 33.25us)         4
       total  |  22.00ms ( 22.00ms)  21.67ms ( 21.67ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  62.00us ( 20.67us)         3
       nlp_g  |        0 (       0)  35.00us ( 11.67us)         3
  nlp_grad_f  |        0 (       0) 143.00us ( 35.75us)         4
  nlp_hess_l  |        0 (       0) 508.00us (254.00us)         2
   nlp_jac_g  |        0 (       0) 111.00us ( 27.75us)         4
       total  |   4.00ms (  4.00ms)   5.19ms (  5.19ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  44.00us ( 14.67us)         3
       nlp_g  |        0 (       0)  26.00us (  8.67us)         3
  nlp_grad_f  |        0 (       0) 125.00us ( 31.25us)         4
  nlp_hess_l  |        0 (       0) 468.00us (234.00us)         2
   nlp_jac_g  |        0 (       0)  99.00us ( 24.75us)         4
       total  |   4.00ms (  4.00ms)   4.63ms (  4.63ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  39.00us ( 13.00us)         3
       nlp_g  |        0 (       0)  24.00us (  8.00us)         3
  nlp_grad_f  |        0 (       0) 116.00us ( 29.00us)         4
  nlp_hess_l  |   1.00ms (500.00us) 408.00us (204.00us)         2
   nlp_jac_g  |   1.00ms (250.00us)  93.00us ( 23.25us)         4
       total  |   4.00ms (  4.00ms)   4.34ms (  4.34ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  53.00us ( 17.67us)         3
       nlp_g  |        0 (       0)  32.00us ( 10.67us)         3
  nlp_grad_f  |        0 (       0) 122.00us ( 30.50us)         4
  nlp_hess_l  |   1.00ms (500.00us) 505.00us (252.50us)         2
   nlp_jac_g  |        0 (       0)  97.00us ( 24.25us)         4
       total  |   5.00ms (  5.00ms)   4.83ms (  4.83ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  68.00us ( 22.67us)         3
       nlp_g  |        0 (       0)  39.00us ( 13.00us)         3
  nlp_grad_f  |        0 (       0) 135.00us ( 33.75us)         4
  nlp_hess_l  |        0 (       0) 653.00us (326.50us)         2
   nlp_jac_g  |        0 (       0) 110.00us ( 27.50us)         4
       total  |   5.00ms (  5.00ms)   5.48ms (  5.48ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |   1.00ms (333.33us)  64.00us ( 21.33us)         3
       nlp_g  |        0 (       0)  35.00us ( 11.67us)         3
  nlp_grad_f  |        0 (       0) 136.00us ( 34.00us)         4
  nlp_hess_l  |        0 (       0) 466.00us (233.00us)         2
   nlp_jac_g  |        0 (       0) 105.00us ( 26.25us)         4
       total  |   5.00ms (  5.00ms)   4.99ms (  4.99ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  68.00us ( 22.67us)         3
       nlp_g  |        0 (       0)  43.00us ( 14.33us)         3
  nlp_grad_f  |        0 (       0) 132.00us ( 33.00us)         4
  nlp_hess_l  |        0 (       0) 497.00us (248.50us)         2
   nlp_jac_g  |        0 (       0) 118.00us ( 29.50us)         4
       total  |   5.00ms (  5.00ms)   5.49ms (  5.49ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |   1.00ms (333.33us)  90.00us ( 30.00us)         3
       nlp_g  |        0 (       0)  47.00us ( 15.67us)         3
  nlp_grad_f  |        0 (       0) 212.00us ( 53.00us)         4
  nlp_hess_l  |   1.00ms (500.00us) 861.00us (430.50us)         2
   nlp_jac_g  |   1.00ms (250.00us) 165.00us ( 41.25us)         4
       total  |   7.00ms (  7.00ms)   7.24ms (  7.24ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  75.00us ( 25.00us)         3
       nlp_g  |        0 (       0)  46.00us ( 15.33us)         3
  nlp_grad_f  |        0 (       0) 149.00us ( 37.25us)         4
  nlp_hess_l  |   2.00ms (  1.00ms) 639.00us (319.50us)         2
   nlp_jac_g  |        0 (       0) 118.00us ( 29.50us)         4
       total  |   6.00ms (  6.00ms)   5.93ms (  5.93ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  38.00us ( 12.67us)         3
       nlp_g  |        0 (       0)  24.00us (  8.00us)         3
  nlp_grad_f  |        0 (       0) 112.00us ( 28.00us)         4
  nlp_hess_l  |        0 (       0) 406.00us (203.00us)         2
   nlp_jac_g  |        0 (       0)  92.00us ( 23.00us)         4
       total  |   4.00ms (  4.00ms)   4.44ms (  4.44ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  64.00us ( 21.33us)         3
       nlp_g  |        0 (       0)  43.00us ( 14.33us)         3
  nlp_grad_f  |        0 (       0) 155.00us ( 38.75us)         4
  nlp_hess_l  |   1.00ms (500.00us) 619.00us (309.50us)         2
   nlp_jac_g  |        0 (       0) 109.00us ( 27.25us)         4
       total  |   5.00ms (  5.00ms)   5.03ms (  5.03ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  54.00us ( 18.00us)         3
       nlp_g  |        0 (       0)  31.00us ( 10.33us)         3
  nlp_grad_f  |        0 (       0) 142.00us ( 35.50us)         4
  nlp_hess_l  |        0 (       0) 518.00us (259.00us)         2
   nlp_jac_g  |        0 (       0) 109.00us ( 27.25us)         4
       total  |   5.00ms (  5.00ms)   4.96ms (  4.96ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  43.00us ( 14.33us)         3
       nlp_g  |        0 (       0)  25.00us (  8.33us)         3
  nlp_grad_f  |        0 (       0) 117.00us ( 29.25us)         4
  nlp_hess_l  |        0 (       0) 417.00us (208.50us)         2
   nlp_jac_g  |        0 (       0) 104.00us ( 26.00us)         4
       total  |   4.00ms (  4.00ms)   4.63ms (  4.63ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  48.00us ( 16.00us)         3
       nlp_g  |        0 (       0)  28.00us (  9.33us)         3
  nlp_grad_f  |        0 (       0) 122.00us ( 30.50us)         4
  nlp_hess_l  |        0 (       0) 414.00us (207.00us)         2
   nlp_jac_g  |        0 (       0)  99.00us ( 24.75us)         4
       total  |   5.00ms (  5.00ms)   4.83ms (  4.83ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  75.00us ( 25.00us)         3
       nlp_g  |        0 (       0)  48.00us ( 16.00us)         3
  nlp_grad_f  |        0 (       0) 168.00us ( 42.00us)         4
  nlp_hess_l  |        0 (       0) 565.00us (282.50us)         2
   nlp_jac_g  |   1.00ms (250.00us) 120.00us ( 30.00us)         4
       total  |   5.00ms (  5.00ms)   5.80ms (  5.80ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  58.00us ( 19.33us)         3
       nlp_g  |        0 (       0)  37.00us ( 12.33us)         3
  nlp_grad_f  |        0 (       0) 179.00us ( 44.75us)         4
  nlp_hess_l  |   1.00ms (500.00us) 625.00us (312.50us)         2
   nlp_jac_g  |        0 (       0) 135.00us ( 33.75us)         4
       total  |   6.00ms (  6.00ms)   6.09ms (  6.09ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  59.00us ( 19.67us)         3
       nlp_g  |        0 (       0)  39.00us ( 13.00us)         3
  nlp_grad_f  |        0 (       0) 146.00us ( 36.50us)         4
  nlp_hess_l  |        0 (       0) 389.00us (194.50us)         2
   nlp_jac_g  |        0 (       0) 122.00us ( 30.50us)         4
       total  |   6.00ms (  6.00ms)   5.51ms (  5.51ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  78.00us ( 26.00us)         3
       nlp_g  |        0 (       0)  42.00us ( 14.00us)         3
  nlp_grad_f  |        0 (       0) 155.00us ( 38.75us)         4
  nlp_hess_l  |   2.00ms (  1.00ms) 693.00us (346.50us)         2
   nlp_jac_g  |        0 (       0) 137.00us ( 34.25us)         4
       total  |   6.00ms (  6.00ms)   6.32ms (  6.32ms)         1
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |        0 (       0)  39.00us ( 13.00us)         3
       nlp_g  |        0 (       0)  23.00us (  7.67us)         3
  nlp_grad_f  |        0 (       0) 105.00us ( 26.25us)         4
  nlp_hess_l  |        0 (       0) 396.00us (198.00us)         2
   nlp_jac_g  |        0 (       0)  85.00us ( 21.25us)         4
       total  |   4.00ms (  4.00ms)   4.21ms (  4.21ms)         1
```
# Plot results
```matlab
time = 0:Ts:n_steps*Ts;

figure;
subplot(2,1,1);
plot(time, x_sim(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(time, x_sim(2,:), 'b', 'LineWidth', 1.5);
legend('Alpha', 'q'); title('State Trajectories'); grid on;

subplot(2,1,2);
stairs(time(1:end-1), u_sim, 'k', 'LineWidth', 1.5);
title('Control Input'); xlabel('Time (s)'); ylabel('Fin Deflection'); grid on;
```

![](./model_predictive_control_final_media//)
# 3D Trajectory Visualization
```matlab
y_pos = zeros(size(x_pos));  % 2D path
figure;
plot3(x_pos/1000, y_pos/1000, z_pos/1000, 'b', 'LineWidth', 1.5); hold on;
plot3(x_pos(1)/1000, 0, z_pos(1)/1000, 'go', 'MarkerSize', 10, 'DisplayName','Start');
plot3(x_pos(end)/1000, 0, z_pos(end)/1000, 'ro', 'MarkerSize', 10, 'DisplayName','End');
xlabel('x (km)'); ylabel('y (km)'); zlabel('z (km)');
title('Estimated 3D Missile Trajectory to Target'); legend; grid on;
```

![](./model_predictive_control_final_media//)
# 3D Missile Animation
```matlab
% Missile body dimensions (in km)
missile_length = 0.1; % 100 meters
missile_radius = 0.02; % 20 meters

% Create cone-shaped missile geometry
[cone_x, cone_y, cone_z] = cylinder([0 missile_radius], 20);
cone_z = missile_length * cone_z;

% Create figure
figure;
hold on;
axis equal;
grid on;
xlabel('x (km)'); ylabel('y (km)'); zlabel('z (km)');
title('Missile Trajectory Animation');
view(3);

% Plot trail
h = plot3(x_pos(1)/1000, y_pos(1)/1000, z_pos(1)/1000, 'b', 'LineWidth', 1.5);

% Plot start and end
plot3(x_pos(1)/1000, y_pos(1)/1000, z_pos(1)/1000, 'go', 'MarkerSize', 8, 'DisplayName','Start');
plot3(x_pos(end)/1000, y_pos(end)/1000, z_pos(end)/1000, 'ro', 'MarkerSize', 8, 'DisplayName','End');

% Set up missile surface (initially hidden)
missile_surf = surf(nan(size(cone_x)), nan(size(cone_y)), nan(size(cone_z)), ...
    'FaceColor', 'r', 'EdgeColor', 'none');

% Loop through trajectory
for k = 2:length(x_pos)
    % Missile center position (in km)
    center = [x_pos(k), y_pos(k), z_pos(k)] / 1000;

    % Get current pitch angle
    theta = pitch_angle(k);  % already in radians

    % Rotation matrix (around y-axis)
    R = [cos(theta) 0 sin(theta);
         0          1        0;
        -sin(theta) 0 cos(theta)];

    % Rotate and translate cone
    rotated = R * [cone_x(:)'; cone_y(:)'; cone_z(:)'];
    Xr = reshape(rotated(1,:) + center(1), size(cone_x));
    Yr = reshape(rotated(2,:) + center(2), size(cone_y));
    Zr = reshape(rotated(3,:) + center(3), size(cone_z));

    % Update missile surface
    set(missile_surf, 'XData', Xr, 'YData', Yr, 'ZData', Zr);

    % Update trail
    set(h, 'XData', x_pos(1:k)/1000, ...
           'YData', y_pos(1:k)/1000, ...
           'ZData', z_pos(1:k)/1000);

    drawnow;
    pause(0.03);
end
```

![](./model_predictive_control_final_media//)
