clc; close all; clear all;
format short

%% Missile Model (from your code)
A = [-1.064 1.000; 290.26 0.00];
B = [-0.25; -331.40];
C = [-123.34 0.00; 0.00 1.00];
D = [-13.51; 0];

states = {'AoA', 'q'};
inputs = {'\delta_c'};
outputs = {'Az','q'};

sys = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

% LQR Controller
Q = [0.1 0; 0 0.1];
R = 0.5; 
[K,S,e] = lqr(A,B,Q,R);
Acl = A-B*K;
Bcl = B;
syscl = ss(Acl,Bcl,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

%% Simulation Parameters
Vel = 1021.08; % m/s (3350ft/s)
simTime = 90; % seconds
dt = 0.01; % time step
t = 0:dt:simTime;

%% Initial and Target Positions
% Convert lat/lon to Cartesian coordinates (approximate for small distances)
earthRadius = 6371000; % meters

% Target location
LAT_TARGET = 34.6588;
LON_TARGET = -118.769745;
ELEV_TARGET = 795; % m - MSL

% Initial location
LAT_INIT = 34.2329;
LON_INIT = -119.4573;
ELEV_INIT = 10000; % m - MSL

% Convert to Cartesian coordinates (local tangent plane approximation)
lat0 = LAT_INIT * pi/180;
lon0 = LON_INIT * pi/180;
lat1 = LAT_TARGET * pi/180;
lon1 = LON_TARGET * pi/180;

% ENU (East-North-Up) coordinates relative to initial position
x0 = 0;
y0 = 0;
z0 = ELEV_INIT;

% Target position relative to initial position
dx = earthRadius * (lon1 - lon0) * cos(lat0);
dy = earthRadius * (lat1 - lat0);
dz = ELEV_TARGET - ELEV_INIT;

target_pos = [dx; dy; dz];

%% Guidance Law (Proportional Navigation)
N = 3; % Navigation constant
lambda = atan2(dy, dx); % Initial line-of-sight angle
theta = lambda; % Initial missile heading (assumed aligned with LOS initially)

% Initial state
pos = [x0; y0; z0]; % Initial position
vel = Vel * [cos(theta); sin(theta); -dz/norm([dx,dy,dz])]; % Initial velocity

% Initialize arrays for logging
pos_history = zeros(3, length(t));
vel_history = zeros(3, length(t));
acc_history = zeros(3, length(t));
miss_distance_history = zeros(1, length(t));

%% Main Simulation Loop
for i = 1:length(t)
    % Current missile to target vector
    r = target_pos - pos;
    miss_distance = norm(r);
    miss_distance_history(i) = miss_distance;
    
    % Line-of-sight rate calculation
    if i > 1
        los = r/norm(r);
        los_prev = (target_pos - pos_history(:,i-1))/norm(target_pos - pos_history(:,i-1));
        los_rate = (los - los_prev)/dt;
    else
        los_rate = [0; 0; 0];
    end
    
    % Proportional Navigation Command
    acc_cmd = N * Vel * cross([0;0;1], los_rate); % Only horizontal for simplicity
    
    % Convert acceleration command to missile dynamics (simplified)
    % This is where your missile model would come into play
    delta_c = -K * [0; 0]; % Placeholder - would use actual states
    
    % Update missile dynamics (simplified 6DOF)
    acc = acc_cmd - [0; 0; 9.81]; % Include gravity
    
    % Update velocity and position
    vel = vel + acc * dt;
    pos = pos + vel * dt;
    
    % Store history
    pos_history(:,i) = pos;
    vel_history(:,i) = vel;
    acc_history(:,i) = acc;
    
    disp(miss_distance);
    % Check for impact
    if miss_distance < 1e3 % meters
        fprintf('Target hit at time = %.2f seconds\n', t(i));
        break;
    end
end

%% 3D Trajectory Plot
figure('Name', '3D Missile Trajectory', 'Position', [100 100 800 600]);
plot3(pos_history(1,1:i), pos_history(2,1:i), pos_history(3,1:i), 'b-', 'LineWidth', 2);
hold on;
plot3(target_pos(1), target_pos(2), target_pos(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(pos_history(1,1), pos_history(2,1), pos_history(3,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

% Add labels and legend
xlabel('East (m)');
ylabel('North (m)');
zlabel('Altitude (m)');
title('3D Missile Trajectory - Final Approach');
legend('Missile Path', 'Target', 'Launch Point', 'Location', 'best');
grid on;
view(45, 30); % Set viewing angle

% Add coordinate axes
line([0 1000], [0 0], [0 0], 'Color', 'k', 'LineStyle', '--');
line([0 0], [0 1000], [0 0], 'Color', 'k', 'LineStyle', '--');
line([0 0], [0 0], [0 1000], 'Color', 'k', 'LineStyle', '--');

%% Miss Distance Plot
figure;
plot(t(1:i), miss_distance_history(1:i), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Miss Distance (m)');
title('Miss Distance vs Time');
grid on;

%% Velocity and Acceleration Profiles
figure;
subplot(2,1,1);
plot(t(1:i), vecnorm(vel_history(:,1:i)), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Missile Velocity vs Time');
grid on;

subplot(2,1,2);
plot(t(1:i), vecnorm(acc_history(:,1:i)), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Missile Acceleration vs Time');
grid on;