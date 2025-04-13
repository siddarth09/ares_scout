clc; close all; clear all;
format short

%% Missile Linearized Dynamics (State-Space)
A = [-1.064   1.000; 
      290.26   0.00];
B = [-0.25; 
     -331.40];
C = [-123.34   0.00; 
        0.00    1.00];
D = [-13.51; 
       0.00];

states = {'AoA', 'q'};       % Angle of Attack (rad), Pitch Rate (rad/s)
inputs = {'\delta_c'};       % Fin Deflection (rad)
outputs = {'Az', 'q'};       % Acceleration (m/s²), Pitch Rate (rad/s)

sys = ss(A, B, C, D, 'statename', states, 'inputname', inputs, 'outputname', outputs);

%% LQR Controller Design
Q = diag([10, 1]);          % State weights: [AoA, q]
R = 1;                    % Control effort weight
[K, ~, ~] = lqr(A, B, Q, R); % Optimal gain matrix

%% Simulation Parameters
Vel = 1021.08;              % Missile velocity (m/s)
simTime = 150;               % Simulation time (s)
dt = 0.01;                  % Time step (s)
t = 0:dt:simTime;

%% Target and Initial Conditions (ENU Coordinates)
earthRadius = 6371000;      % Earth radius (m)

% Target location (34.6588°N, -118.769745°W, 795m altitude)
target_lla = [34.6588, -118.769745, 795];  

% Initial location (34.2329°N, -119.4573°W, 10000m altitude)
init_lla = [34.2329, -119.4573, 10000];  

% Convert to ENU (East-North-Up) coordinates
lat0 = init_lla(1)*pi/180;
lon0 = init_lla(2)*pi/180;
lat1 = target_lla(1)*pi/180;
lon1 = target_lla(2)*pi/180;

% Relative positions
dx = earthRadius * (lon1 - lon0) * cos(lat0);
dy = earthRadius * (lat1 - lat0);
dz = target_lla(3) - init_lla(3);

target_pos = [dx; dy; dz];  % Target position (ENU)
init_pos = [0; 0; init_lla(3)]; % Initial position (ENU)

%% Initial State
pos = init_pos;             % Initial position (m)
theta = atan2(dy, dx);      % Initial heading (rad)
phi = atan2(-dz, sqrt(dx^2 + dy^2)); % Initial elevation (rad)
vel = Vel * [cos(theta)*cos(phi); sin(theta)*cos(phi); -sin(phi)]; % Initial velocity (m/s)
x = [0; 0];                 % Initial state: [AoA; q] (rad, rad/s)

%% LQR Reference Calculation
% Desired AoA is set to drive the missile toward the target
% Simple heuristic: AoA_ref ~ -k * (missile-to-target angle error)
k_guidance = 0.025;          % Guidance gain

%% Logging Variables
pos_history = zeros(3, length(t));
vel_history = zeros(3, length(t));
AoA_history = zeros(1, length(t));
q_history = zeros(1, length(t));
delta_c_history = zeros(1, length(t));
az_history = zeros(1, length(t));

for i = 1:length(t)
    % Current missile-to-target vector
    r = target_pos - pos;
    miss_distance = norm(r);
    
    % Calculate angle error (simplified guidance)
    theta_target = atan2(r(2), r(1)); % Desired direction to target
    theta_error = mod(theta_target - theta + pi, 2*pi) - pi; % Normalize to [-pi, pi]

    phi_error = atan2(-r(3), sqrt(r(1)^2 + r(2)^2)) - phi;
    
    % LQR Reference (AoA_ref based on angle error)
    alpha_ref = -k_guidance * (theta_error + phi_error);
    x_ref = [alpha_ref; 0];  % No reference for q
    
    %% LQR Control Law
    delta_c = -K * (x - x_ref); % Fin deflection (rad)
    delta_c = max(min(delta_c, 0.35), -0.35); % Saturate at ±20°
    disp(delta_c);

    %% Missile Dynamics (State-space update)
    x_dot = A * x + B * delta_c;
    x = x + x_dot * dt;     % Euler integration
    
    % Output equation (Az and q)
    Az = C(1,:) * x + D(1) * delta_c; % Lateral acceleration (m/s²)
    
    %% Update Kinematics (Simplified 6DOF)
    % Convert Az to body-frame acceleration
    acc_body = [0; Az; 0]; % Include gravity
    
    % Rotate to ENU frame (simplified)
    acc_ENU = [cos(theta)*cos(phi), -sin(theta), -cos(theta)*sin(phi);
               sin(theta)*cos(phi),  cos(theta), -sin(theta)*sin(phi);
               sin(phi),            0,           cos(phi)] * acc_body;
    
    % Update velocity and position
    vel = vel + acc_ENU * dt;
    pos = pos + vel * dt;
    
    % Update heading and elevation angles (simplified)
    theta = atan2(vel(2), vel(1));
    phi = atan2(-vel(3), sqrt(vel(1)^2 + vel(2)^2));
    
    %% Store results
    pos_history(:,i) = pos;
    vel_history(:,i) = vel;
    AoA_history(i) = x(1);
    q_history(i) = x(2);
    delta_c_history(i) = delta_c;
    az_history(i) = Az;
    
    disp(miss_distance);
    %% Check for impact
    if miss_distance < 1e4
        fprintf('Target hit at t = %.2f s\n', t(i));
        break;
    end
end

%% Trim unused simulation points
sim_steps = i;
t = t(1:sim_steps);
pos_history = pos_history(:,1:sim_steps);
vel_history = vel_history(:,1:sim_steps);
AoA_history = AoA_history(1:sim_steps);
q_history = q_history(1:sim_steps);
delta_c_history = delta_c_history(1:sim_steps);
az_history = az_history(1:sim_steps);

%% Plot Results
% 3D Trajectory
figure(1);
plot3(pos_history(1,:), pos_history(2,:), pos_history(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(target_pos(1), target_pos(2), target_pos(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(init_pos(1), init_pos(2), init_pos(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('East (m)'); ylabel('North (m)'); zlabel('Altitude (m)');
title('3D Missile Trajectory (Pure LQR Control)');
legend('Missile Path', 'Target', 'Launch Point', 'Location', 'best');
grid on; view(45, 30);

% LQR Control Performance
figure(2);
subplot(3,1,1);
plot(t, AoA_history*180/pi);
ylabel('AoA (deg)'); title('Angle of Attack');

subplot(3,1,2);
plot(t, delta_c_history*180/pi);
ylabel('\delta_c (deg)'); title('Fin Deflection');

subplot(3,1,3);
plot(t, az_history/9.81);
ylabel('A_z (g)'); title('Lateral Acceleration');
xlabel('Time (s)');

% Miss Distance
figure(3);
miss_distance = vecnorm(target_pos - pos_history);
plot(t, miss_distance);
xlabel('Time (s)'); ylabel('Miss Distance (m)');
title('Miss Distance vs Time');
grid on;

%% Improved State Trajectories and Control Input Plots
figure('Name', 'State Trajectories and Control Input', 'Position', [100, 100, 900, 700]);

% State trajectories (AoA and q)
subplot(3,1,1);
plot(t, AoA_history*180/pi, 'r-', 'LineWidth', 1.5); hold on;
plot(t, q_history*180/pi, 'b-', 'LineWidth', 1.5);
ylabel('Angle (deg)');
legend('Angle of Attack', 'Pitch Rate', 'Location', 'best');
title('State Trajectories');
grid on;
ylim([-30, 30]); % Set reasonable y-axis limits

% Control input (fin deflection)
subplot(3,1,2);
plot(t, delta_c_history*180/pi, 'k-', 'LineWidth', 1.5);
ylabel('Fin Deflection (deg)');
title('Control Input');
grid on;
ylim([-25, 25]); % Set reasonable y-axis limits

% Acceleration
subplot(3,1,3);
plot(t, az_history/9.81, 'g-', 'LineWidth', 1.5);
ylabel('Acceleration (g)');
xlabel('Time (s)');
title('Lateral Acceleration');
grid on;
ylim([-3, 3]); % Set reasonable y-axis limits

% Adjust overall appearance
set(gcf, 'Color', 'white'); % White background
set(findall(gcf, 'type', 'axes'), 'FontSize', 12, 'LineWidth', 1, 'Box', 'on');

% Save the figure with high resolution
print('state_trajectories_control_input', '-dpng', '-r300');