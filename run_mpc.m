function [x_pos, z_pos, pitch_angle, u_sim] = run_mpc(current_window, Ad, Bd, Cd, Dd, nx, nu, Qy, R, Vel, Ts, f, x_start, z_start)
    % run_mpc - Runs the MPC loop for a given segment of the trajectory
    % current_window - An Nx2 matrix containing [x, z] coordinates of the current trajectory window

    import casadi.*
    
    %% Simulation parameters
    N = 10;  % Prediction horizon
    n_steps = 50;
    x0_val = [0; 0];

    x_sim = zeros(nx, n_steps+1); u_sim = zeros(nu, n_steps);
    x_sim(:,1) = x0_val;

    x_pos = zeros(1, n_steps+1);
    z_pos = zeros(1, n_steps+1);
    pitch_angle = zeros(1, n_steps+1);

    % Starting point for this segment
    x_pos(1) = x_start;
    z_pos(1) = z_start;

    U = MX.sym('U', nu, N);
    X = MX.sym('X', nx, N+1);
    x0 = MX.sym('x0', nx);

    %% Reference tracking
    Az_step = linspace(0, 15, N);  % Target normal acceleration
    q_ref = zeros(1, N);           % Target pitch rate
    y_ref = [Az_step; q_ref];
    
    % Extract the final point from the current window to serve as the target
    current_target = current_window(end, :)';  % Extract the last point of the window as the target

    for t = 1:n_steps
        cost = 0; g = {};
        g{end+1} = X(:,1) - x0;

        pitch_pred = MX.zeros(1, N+1);
        pitch_pred(1) = pitch_angle(t);

        x_pos_pred = MX.zeros(1, N+1);
        z_pos_pred = MX.zeros(1, N+1);

        x_pos_pred(1) = x_pos(t);
        z_pos_pred(1) = z_pos(t);

        for k = 1:N
            pitch_pred(k+1) = pitch_pred(k) + X(2,k)*Ts;
            x_k = X(:,k); u_k = U(:,k);
            x_next = f(x_k, u_k);
            g{end+1} = X(:,k+1) - x_next;

            y_k = Cd * x_k;
            y_ref_k = y_ref(:,k);
            cost = cost + (y_k - y_ref_k)' * Qy * (y_k - y_ref_k) + u_k' * R * u_k;

            x_pos_pred(k+1) = x_pos_pred(k) + Vel * cos(pitch_pred(k+1)) * Ts;
            z_pos_pred(k+1) = z_pos_pred(k) + Vel * sin(pitch_pred(k+1)) * Ts;
        end

        % Compute the final position predicted by the MPC
        final_pos = [x_pos_pred(end); z_pos_pred(end)];
        Qterm = 0.01 * eye(2);

        % Compute the cost for reaching the final point of the window
        cost = cost + (final_pos - current_target)' * Qterm * (final_pos - current_target);

        % Setup and solve the optimization problem
        opt_vars = [reshape(X, nx*(N+1), 1); reshape(U, nu*N, 1)];
        nlp = struct('x', opt_vars, 'f', cost, 'g', vertcat(g{:}), 'p', x0);
        
        solver = nlpsol('solver', 'ipopt', nlp, struct('ipopt', struct('print_level', 0, 'max_iter', 50, 'tol', 1e-2)));
        
        U0 = zeros(nu, N); 
        X0 = repmat(x_sim(:,t), 1, N+1); 
        w0 = [X0(:); U0(:)];
        
        sol = solver('x0', w0, 'p', x_sim(:,t), 'lbg', 0, 'ubg', 0);
        w_opt = full(sol.x);
        U_opt = reshape(w_opt(nx*(N+1)+1:end), nu, N);

        % Apply control input and update the states
        u0 = U_opt(:,1); 
        u_sim(:,t) = u0;
        
        x_sim(:,t+1) = Ad * x_sim(:,t) + Bd * u0;

        % Update position and angle for the next step
        pitch_angle(t+1) = pitch_angle(t) + x_sim(2,t) * Ts;  % Update pitch angle
        x_pos(t+1) = x_pos(t) + Vel * cos(pitch_angle(t+1)) * Ts;
        z_pos(t+1) = z_pos(t) + Vel * sin(pitch_angle(t+1)) * Ts;
    end
end
