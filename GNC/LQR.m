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

%% Pure LQR Design (No Integral)
Qy = diag([1, 10]);        % Output tracking emphasis (Az and q)
Q = Cd' * Qy * Cd;         % Projected to state space
R = 0.0001;
K = dlqr(Ad, Bd, Q, R);

%% Simulation setup
n_steps = 100;
x_sim = zeros(2, n_steps+1); 
x_sim(:,1) = [0; 0];
u_sim = zeros(1, n_steps);

% Reference signals
Az_ref = [linspace(10, 15, 10), 15*ones(1, n_steps-10)];
q_ref = zeros(1, n_steps);
y_ref = [Az_ref; q_ref];

%% Trajectory setup
Vel = 1021.08;
pitch_angle = zeros(1,n_steps+1);
x_pos = zeros(1,n_steps+1);
z_pos = zeros(1,n_steps+1);
x_pos(1) = 1590;
z_pos(1) = 13000;

%% Simulation loop
for t = 1:n_steps
    % Output tracking error
    y = C * x_sim(:,t);
    y_desired = y_ref(:,t);
    error = Cd' * (y - y_desired);

    % Control input (pure state feedback)
    u = -K * x_sim(:,t);
    u_sim(t) = u;

    % State update
    x_sim(:,t+1) = Ad * x_sim(:,t) + Bd * u;

    % Trajectory update
    pitch_angle(t+1) = pitch_angle(t) + x_sim(2,t)*Ts;
    x_pos(t+1) = x_pos(t) + Vel*cos(pitch_angle(t+1))*Ts;
    z_pos(t+1) = z_pos(t) + Vel*sin(pitch_angle(t+1))*Ts;
end

%% 3D Trajectory Visualization
time = 0:Ts:n_steps*Ts;
y_pos = zeros(size(x_pos));
figure;
plot3(x_pos/1000, y_pos/1000, z_pos/1000, 'b'); hold on;
plot3(x_pos(1)/1000, 0, z_pos(1)/1000, 'go', 'MarkerSize', 10);
plot3(x_pos(end)/1000, 0, z_pos(end)/1000, 'ro', 'MarkerSize', 10);
xlabel('x (km)'); ylabel('y (km)'); zlabel('z (km)');
title('3D Missile Trajectory - Pure LQR'); grid on;

%% Performance Analysis
t = 0:Ts:n_steps*Ts;
altitude_error = abs(z_pos - 3000);
[~, reach_idx] = min(altitude_error);
time_to_target = t(reach_idx);
fprintf('Time to Reach Target Altitude: %.2f seconds\n', time_to_target);

final_state = x_sim(:, end);
final_output = C * final_state;
final_error = y_ref(:, end) - final_output;
fprintf('Final Tracking Errors:\n');
fprintf('  Az Error: %.2f\n', final_error(1));
fprintf('   q Error: %.2f\n', final_error(2));

total_control_effort = sum(abs(u_sim(:)));
fprintf('Total Control Effort: %.2f\n', total_control_effort);
average_control = mean(abs(u_sim(:)));
fprintf('Average Control Input Magnitude: %.2f\n', average_control);
max_control = max(abs(u_sim(:)));
fprintf('Maximum Control Input: %.2f\n', max_control);

%% Control Input Histogram
figure;
histogram(u_sim(:), 20);
title('Control Input Distribution - Pure LQR');
xlabel('Fin Deflection'); ylabel('Frequency'); grid on;

%% Tracking Error Plot
tracking_error = zeros(2, n_steps);
for i = 1:n_steps
    y_i = C * x_sim(:, i);
    if i <= size(y_ref, 2)
        ref_i = y_ref(:, i);
    else
        ref_i = y_ref(:, end);
    end
    tracking_error(:, i) = y_i - ref_i;
end

figure;
subplot(2,1,1);
plot(t(1:end-1), tracking_error(1,:), 'r', 'LineWidth', 1.5);
title('Az Tracking Error - Pure LQR'); ylabel('Error (g)'); grid on;

subplot(2,1,2);
plot(t(1:end-1), tracking_error(2,:), 'b', 'LineWidth', 1.5);
title('Pitch Rate Tracking Error - Pure LQR'); xlabel('Time (s)'); ylabel('Error'); grid on;
