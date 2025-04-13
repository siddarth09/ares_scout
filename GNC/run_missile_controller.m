function metrics = run_missile_controller(controller_type)
% RUN_MISSILE_CONTROLLER - Runs MPC, LQR, or SFI missile autopilot
% Inputs:
%   controller_type - 'LQR', 'SFI', or 'MPC'
% Outputs:
%   metrics - Struct containing trajectory and performance analysis

% Simulation Parameters
Ts = 0.1;
n_steps = 100;
Vel = 1021.08;
C = [-123.34 0; 0 1];  % Output matrix for Az and q

% Initial Conditions
x_sim = zeros(2, n_steps+1); 
x_sim(:,1) = [0; 0];
u_sim = zeros(1, n_steps);
pitch_angle = zeros(1,n_steps+1);
x_pos = zeros(1,n_steps+1); x_pos(1) = 1590;
z_pos = zeros(1,n_steps+1); z_pos(1) = 13000;
Az_ref = [linspace(10, 15, 10), 15*ones(1, n_steps-10)];
q_ref = zeros(1, n_steps);
y_ref = [Az_ref; q_ref];

%% Controller Logic
switch controller_type
    case 'LQR'
        A = [-1.064 1; 290.26 0]; B = [-0.25; -331.40];
        sysd = c2d(ss(A,B,C,zeros(2,1)), Ts);
        Ad = sysd.A; Bd = sysd.B; Cd = sysd.C;
        A_aug = [Ad, zeros(2,1); -Cd(1,:)*Ts, 1];
        B_aug = [Bd; 0];
        Qy = diag([1,10]); R = 0.0001;
        Q_aug = blkdiag(Cd'*Qy*Cd, 1000);
        K_aug = dlqr(A_aug, B_aug, Q_aug, R);
        int_e = 0;

        for t = 1:n_steps
            x_aug = [x_sim(:,t); int_e];
            u = -K_aug * x_aug; u_sim(t) = u;
            x_sim(:,t+1) = Ad * x_sim(:,t) + Bd * u;
            Az = Cd(1,:) * x_sim(:,t);
            int_e = int_e + (Az_ref(t) - Az) * Ts;
            pitch_angle(t+1) = pitch_angle(t) + x_sim(2,t)*Ts;
            x_pos(t+1) = x_pos(t) + Vel*cos(pitch_angle(t+1))*Ts;
            z_pos(t+1) = z_pos(t) + Vel*sin(pitch_angle(t+1))*Ts;
        end

    case 'SFI'
        % Missile Dynamics (SFI)
        A = [-1.064 1; 290.26 0];
        B = [-0.25; -331.40];
        C1 = [-123.34 0];  % Az output only
        A_aug = [A, zeros(2,1); -C1, 0];
        B_aug = [B; 0];
        poles = [-4, -6, -8];
        K_aug = place(A_aug, B_aug, poles);
        K = K_aug(1:2); Ki = K_aug(3);
        
        % LOS targeting parameters
        X_target = 10500;
        Z_target = 3000;
        int_e = 0;
    
        for t = 1:n_steps
            dx = X_target - x_pos(t);
            dz = Z_target - z_pos(t);
            LOS_angle = atan2(dz, dx);
            LOS_error = max(min(LOS_angle - pitch_angle(t), deg2rad(20)), deg2rad(-20));
    
            Az_ref_t = -10 * LOS_error;
            if t * Ts < 2
                Az_ref_t = Az_ref_t + 11.3;
            end
    
            Az = C1 * x_sim(:,t);
            e = Az_ref_t - Az;
            int_e = int_e + e * Ts;
    
            u = -K * x_sim(:,t) - Ki * int_e;
            u = max(min(u, 0.3), -0.3);  % Saturation
            u_sim(t) = u;
    
            x_sim(:,t+1) = A * x_sim(:,t) + B * u;
            pitch_angle(t+1) = pitch_angle(t) + x_sim(2,t) * Ts;
    
            vx = 900 * cos(pitch_angle(t));
            vz = 900 * sin(pitch_angle(t));
            x_pos(t+1) = x_pos(t) + vx * Ts;
            z_pos(t+1) = z_pos(t) + vz * Ts;
       end
    
    case 'MPC'
        % Load symbolic model (must be pre-initialized in workspace)
        import casadi.*
        A = [-1.064 1; 290.26 0]; B = [-0.25; -331.40]; C = [-123.34 0; 0 1];
        Ts = 0.1; n_steps = 100; N = 15;
        sysd = c2d(ss(A,B,C,zeros(2,1)),Ts);
        Ad = sysd.A; Bd = sysd.B; Cd = sysd.C;
        nx = size(Ad,1); nu = size(Bd,2);

        x = MX.sym('x',nx); u = MX.sym('u',nu);
        x_next = Ad*x + Bd*u;
        f = Function('f',{x,u},{x_next});

        for t = 1:n_steps
            U = MX.sym('U',nu,N);
            X = MX.sym('X',nx,N+1);
            x0 = MX.sym('x0',nx);
            cost = 0; g = {}; g{end+1} = X(:,1) - x0;

            pitch_pred = MX.zeros(1,N+1);
            pitch_pred(1) = pitch_angle(t);
            x_pos_pred = MX.zeros(1,N+1);
            z_pos_pred = MX.zeros(1,N+1);
            x_pos_pred(1) = x_pos(t);
            z_pos_pred(1) = z_pos(t);

            Qy = diag([25, 10]); R = 1; Qterm = 100*eye(2);
            for k = 1:N
                pitch_pred(k+1) = pitch_pred(k) + X(2,k)*Ts;
                x_k = X(:,k); u_k = U(:,k);
                x_next = f(x_k, u_k);
                g{end+1} = X(:,k+1) - x_next;
                y_k = C * x_k;
                y_ref_k = [Az_ref(min(t+k,n_steps)); 0];
                cost = cost + (y_k - y_ref_k)'*Qy*(y_k - y_ref_k) + u_k'*R*u_k;
                x_pos_pred(k+1) = x_pos_pred(k) + Vel*cos(pitch_pred(k+1))*Ts;
                z_pos_pred(k+1) = z_pos_pred(k) + Vel*sin(pitch_pred(k+1))*Ts;
            end
            final_pos = [x_pos_pred(end); z_pos_pred(end)];
            target = [10500; 3000];
            cost = cost + (final_pos - target)' * Qterm * (final_pos - target);

            opt_vars = [reshape(X, nx*(N+1), 1); reshape(U, nu*N, 1)];
            nlp = struct('x', opt_vars, 'f', cost, 'g', vertcat(g{:}), 'p', x0);
            solver = nlpsol('solver','ipopt',nlp,struct('ipopt',struct('print_level',0)));
            U0 = zeros(nu,N); X0 = repmat(x_sim(:,t),1,N+1); w0 = [X0(:); U0(:)];
            u_min = -0.3; u_max = 0.2;
            lbx = -inf(size(w0)); ubx = inf(size(w0));
            lbx(end-nu*N+1:end) = u_min; ubx(end-nu*N+1:end) = u_max;
            sol = solver('x0', w0, 'p', x_sim(:,t), 'lbg', 0, 'ubg', 0, 'lbx', lbx, 'ubx', ubx);
            w_opt = full(sol.x);
            U_opt = reshape(w_opt(nx*(N+1)+1:end), nu, N);
            u0 = U_opt(:,1); u_sim(t) = u0;
            x_sim(:,t+1) = Ad * x_sim(:,t) + Bd * u0;
            pitch_angle(t+1) = pitch_angle(t) + x_sim(2,t)*Ts;
            x_pos(t+1) = x_pos(t) + Vel * cos(pitch_angle(t+1)) * Ts;
            z_pos(t+1) = z_pos(t) + Vel * sin(pitch_angle(t+1)) * Ts;
        end

    otherwise
        error('Unknown controller type. Use ''LQR'', ''SFI'', or ''MPC''.');
end

%% Performance Analysis
t_vec = 0:Ts:n_steps*Ts;
altitude_error = abs(z_pos - 3000);
[~, reach_idx] = min(altitude_error);
final_output = C * x_sim(:, end);
final_error = y_ref(:, end) - final_output;

metrics = struct();
metrics.type = controller_type;
metrics.time_to_target = t_vec(reach_idx);
metrics.Az_error = final_error(1);
metrics.q_error = final_error(2);
metrics.total_effort = sum(abs(u_sim));
metrics.average_u = mean(abs(u_sim));
metrics.max_u = max(abs(u_sim));
metrics.x_pos = x_pos;
metrics.z_pos = z_pos;
metrics.pitch_angle = pitch_angle;
end
