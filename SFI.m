
% SFI CONTROLLER 

% Missile Dynamics from MPC (same A, B, C matrices)
A = [-1.064 1.000;
     290.26 0.00];
B = [-0.25;
    -331.40];
C = [-123.34 0.00];  % Output: Azm

% Augmented system for integral control
A_aug = [A, zeros(2,1); -C, 0];
B_aug = [B; 0];
% Pole placement
poles = [-4, -6, -8];
K_aug = place(A_aug, B_aug, poles);
K = K_aug(1:2);
Ki = K_aug(3);

% Simulation parameters
dt = 0.01;
T = 13.8;
t = 0:dt:T;
n = length(t);

% Initial conditions
x = zeros(2, n);
int_error = 0;
Azm = zeros(1, n);
u = zeros(1, n);
theta = zeros(1, n);
x_pos = zeros(1, n);
z_pos = zeros(1, n);
x_pos(1) = 0;
z_pos(1) = 3000;
theta(1) = 0;

% Target-based LOS Guidance
X_target = 12000;
Z_target = 0;
K_guidance = 10;

for i = 1:n-1
    dx = X_target - x_pos(i);
    dz = Z_target - z_pos(i);
    LOS_angle = atan2(dz, dx);
    LOS_error = LOS_angle - theta(i);

    % Saturate LOS error
    LOS_error = max(min(LOS_error, deg2rad(20)), deg2rad(-20));

    % Distance to target
    distance = sqrt(dx^2 + dz^2);

    % Adaptive gain near target
    if distance < 1000
        K_guided = 3;
    else
        K_guided = K_guidance;
    end

    % LOS-based reference + initial boost
    Az_ref = -K_guided * LOS_error;
    if t(i) < 2
        Az_ref = Az_ref + 11.3;
    end

    % SFI Controller
    Az = C * x(:, i);
    error = Az_ref - Az;
    int_error = int_error + dt * error;
    u(i) = -K * x(:, i) - Ki * int_error;

    % Dynamics update
    dx_vec = A * x(:, i) + B * u(i);
    x(:, i+1) = x(:, i) + dt * dx_vec;
    Azm(i) = Az;

    % Trajectory update
    theta(i+1) = theta(i) + x(2,i)*dt;
    vx = 900 * cos(theta(i));
    vz = 900 * sin(theta(i));
    x_pos(i+1) = x_pos(i) + vx * dt;
    z_pos(i+1) = z_pos(i) + vz * dt;
end

% Plot trajectory
figure;
plot(x_pos, z_pos, 'b', 'LineWidth', 2); hold on;
plot(X_target, Z_target, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('X Position (m)'); ylabel('Z Altitude (m)');
title('Missile Trajectory Toward Ground Target (SFI + Improved Guidance)');
legend('Missile Path', 'Target'); grid on;

% Plot Azm
figure;
plot(t, Azm, 'm', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Azm (g)');
title('Azm Response (SFI Controller)'); grid on;

% Plot Control Input
figure;
plot(t, u, 'k', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Fin Deflection (\\delta_p)');
title('Control Input (SFI Controller)'); grid on;


figure;
plot(t, x(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(t, x(2,:), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('State Value');
title('Alpha and q vs Time');
legend('Alpha (rad)', 'q (rad/s)');
grid on;

figure;
plot(t, x(2,:), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('q (rad/s)');
title('Pitch Rate (q) vs Time');
grid on;


% Initialize 3D plot
figure;
set(gcf, 'Position', [150, 150, 800, 700]);  
view([-35 25]);   % set to show more of the scene

hold on;
grid on;
axis equal;
axis vis3d;  



view(3);     % Set 3D view

% Set axes labels
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Altitude (m)');
view([135, 30]);     % or try [135, 30] for different angles
camup([0 0 1]);      % Z axis up

title('Missile Trajectory Animation in 3D ');

% Define axis limits to be equal in scale 
min_axis = min([min(x_pos), min(z_pos)]) - 20;
max_axis = max([max(x_pos), max(z_pos)]) + 20;

xlim([min_axis, max_axis]);
ylim([-0.5*(max_axis-min_axis), 0.5*(max_axis-min_axis)]);  % Keep Y small
zmin = min(z_pos) - 200;
zmax = max(z_pos) + 200;
zlim([zmin, zmax]);



% Create Y-axis values (flat trajectory)
y_pos = zeros(size(x_pos));

% Plot target in 3D
hold on;
plot3(X_target, 0, Z_target, 'ro', 'MarkerSize', 8, 'LineWidth', 2);

% Initialize missile marker and trajectory trail
missile_marker = plot3(x_pos(1), y_pos(1), z_pos(1), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
trail = plot3(nan, nan, nan, 'b');

% Animate missile along flat Y-axis
for k = 2:length(x_pos)
    set(missile_marker, 'XData', x_pos(k), 'YData', y_pos(k), 'ZData', z_pos(k));
    set(trail, 'XData', x_pos(1:k), 'YData', y_pos(1:k), 'ZData', z_pos(1:k));
    drawnow;
    pause(0.01);  % Speed of animation
end

% Final annotation
text3(X_target, 0, Z_target + 200, 'Target Reached!', 'Color', 'red', ...
    'FontSize', 12, 'HorizontalAlignment', 'center');


