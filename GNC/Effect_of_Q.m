%% Missile Longitudinal Dynamics
A = [-1.064 1.000; 290.26 0.00];
B = [-0.25; -331.40];
C = [-123.34 0.00; 0.00 1.00];
D = [0; 0];
plant = ss(A,B,C,D);

% Check system properties
if rank(ctrb(A,B)) == size(A,1), disp("System is controllable"), end
if rank(obsv(A,C)) == size(A,1), disp("System is observable"), end

%% Discretization
Ts = 0.1;
sysd = c2d(plant,Ts);
Ad = sysd.A; Bd = sysd.B; Cd = sysd.C; Dd = sysd.D;

%% Augment system with integral state for Az tracking
A_aug = [Ad, zeros(2,1); 
         -Cd(1,:)*Ts, 1];
B_aug = [Bd; 0];

% Check controllability of augmented system
if rank(ctrb(A_aug, B_aug)) == size(A_aug,1)
    disp("Augmented system is controllable")
else
    error("Augmented system is not controllable");
end

%% LQR design for augmented system
Qy = diag([1, 10]);  % Output weighting 
R = 0.0001;                 % Control weighting
disp(Qy);
% Original state weighting 
Q_original = Cd' * Qy * Cd;
disp(Q_original);
% Augmented state weighting (add weight for integral term)
Q_int = 1000;  % Tune this parameter for integral action strength
Q_aug = blkdiag(Q_original, Q_int);
disp(Q_aug)
% Compute LQR gain
[K_aug, ~, ~] = dlqr(A_aug, B_aug, Q_aug, R);

%% Define different Q cases for analysis
cases = struct();

% Case 1: Default weights (original design)
cases(1).Qy = diag([1, 1]);       % Output weights [Az, q]
cases(1).Q_int = 1000;            % Integral weight
cases(1).label = 'Baseline';

% Case 2: Higher penalty on pitch rate (q)
cases(2).Qy = diag([1, 100]);     % 100x higher weight on q 
cases(2).Q_int = 1000;
cases(2).label = 'High q Weight';

% Case 3: Reduced penalty on angle of attack (α)
cases(3).Qy = diag([0.1, 1]);     % 10x lower weight on Az (α-dominated)
cases(3).Q_int = 1000;
cases(3).label = 'Low α Weight';

% Case 4: Aggressive integral action
cases(4).Qy = diag([1, 1]);
cases(4).Q_int = 5000;            % 5x higher integral weight
cases(4).label = 'Strong Integral';

% Case 5: Weak integral action
cases(5).Qy = diag([1, 1]);
cases(5).Q_int = 100;             % 10x lower integral weight
cases(5).label = 'Weak Integral';

%% Simulation parameters
n_steps = 100;  % Number of simulation steps
sim_time = 10;  % Total simulation time (seconds)

% Create reference signal (step input at t=1s)
Az_ref = zeros(n_steps, 1);
step_time = 1.0;  % Step occurs at 1 second
step_idx = ceil(step_time/Ts);
Az_ref(step_idx:end) = 1.0;  % Unit step of 1.0

%% Preallocate storage for results
n_cases = length(cases);
alpha_results = zeros(n_steps+1, n_cases);
q_results = zeros(n_steps+1, n_cases);
u_results = zeros(n_steps, n_cases);

%% Simulate all cases
for case_idx = 1:n_cases
    % Get current Q parameters
    Qy = cases(case_idx).Qy;
    Q_int = cases(case_idx).Q_int;
    
    % Recompute LQR weights
    Q_original = Cd' * Qy * Cd;   % Convert output weights to state weights
    Q_aug = blkdiag(Q_original, Q_int);
    
    % Calculate LQR gain
    [K_aug, ~, ~] = dlqr(A_aug, B_aug, Q_aug, R);
    
    % Reset simulation variables
    x_sim = zeros(2, n_steps+1);
    x_sim(:,1) = [0; 0];  % Initial state
    u_sim = zeros(1, n_steps);
    int_e = 0;             % Integral error reset
    
    % Simulation loop
    for t = 1:n_steps
        % Current reference
        current_Az_ref = Az_ref(t);
        current_Az = Cd(1,:)*x_sim(:,t);
        
        % Augmented state
        x_aug = [x_sim(:,t); int_e];
        
        % Control calculation
        u = -K_aug * x_aug;
        u_sim(t) = u;
        
        % State update
        x_sim(:,t+1) = Ad*x_sim(:,t) + Bd*u;
        
        % Integral update
        int_e = int_e + (current_Az_ref - current_Az)*Ts;
    end
    
    % Store results
    alpha_results(:, case_idx) = x_sim(1,:)';
    q_results(:, case_idx) = x_sim(2,:)';
    u_results(:, case_idx) = u_sim';
end

%% Plot comparison results
time = (0:n_steps)*Ts;

% Angle of attack comparison
figure;
set(gcf,'Position',[100 100 1200 800]);
subplot(3,1,1);
hold on;
colors = lines(n_cases);
for i = 1:n_cases
    plot(time, alpha_results(:,i), 'Color', colors(i,:));
end
ylabel('α (rad)');
title('Angle of Attack Response');
legend({cases.label}, 'Location', 'northeastoutside');
grid on;

% Pitch rate comparison
subplot(3,1,2);
hold on;
for i = 1:n_cases
    plot(time, q_results(:,i), 'Color', colors(i,:));
end
ylabel('q (rad/s)');
title('Pitch Rate Response');
grid on;

% Control effort comparison
subplot(3,1,3);
hold on;
for i = 1:n_cases
    stairs(time(1:end-1), u_results(:,i), 'Color', colors(i,:), 'LineWidth', 1.5);
end
ylabel('δ (rad)');
xlabel('Time (s)');
title('Control Surface Deflection');
grid on;

%% Key observations annotation
annotation('textbox', [0.15 0.05 0.7 0.1], 'String', ...
    ['Key Observations:\n'...
     '1. High q Weight: Smoother pitch rate but slower Az tracking\n'...
     '2. Low α Weight: Larger initial α deviation but faster response\n'...
     '3. Strong Integral: Faster tracking with more aggressive control\n'...
     '4. Weak Integral: Steady-state error persists'], ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'FontSize', 10, 'BackgroundColor', [1 1 0.8]);