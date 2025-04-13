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
Qy = diag([1, 10]);  
R = 0.0001;
Q_original = Cd' * Qy * Cd;
Q_int = 1000;
Q_aug = blkdiag(Q_original, Q_int);
[K_aug, ~, ~] = dlqr(A_aug, B_aug, Q_aug, R);

%% Simulation setup
n_steps = 100;
x_sim = zeros(2, n_steps+1); 
x_sim(:,1) = [0; 0];
u_sim = zeros(1, n_steps);
int_e = 0;

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
    current_Az_ref = Az_ref(t);
    x_aug = [x_sim(:,t); int_e];
    u = -K_aug * x_aug;
    u_sim(t) = u;
    x_sim(:,t+1) = Ad * x_sim(:,t) + Bd * u;

    current_Az = Cd(1,:) * x_sim(:,t);
    int_e = int_e + (current_Az_ref - current_Az) * Ts;

    pitch_angle(t+1) = pitch_angle(t) + x_sim(2,t)*Ts;
    x_pos(t+1) = x_pos(t) + Vel*cos(pitch_angle(t+1))*Ts;
    z_pos(t+1) = z_pos(t) + Vel*sin(pitch_angle(t+1))*Ts;
end
%% Plot results
time = 0:Ts:n_steps*Ts;
figure;
plot(0:Ts:Ts*n_steps, z_pos, 'LineWidth', 1.5);
title('Missile Altitude vs Time'); ylabel('Altitude (m)'); xlabel('Time (s)'); grid on;

figure;
subplot(2,1,1);
plot(time, x_sim(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(time, x_sim(2,:), 'b', 'LineWidth', 1.5);
legend('Alpha', 'q'); title('State Trajectories'); grid on;

subplot(2,1,2);
stairs(time(1:end-1), u_sim, 'k', 'LineWidth', 1.5);
title('Control Input'); xlabel('Time (s)'); ylabel('Fin Deflection'); grid on;

y_pos = zeros(size(x_pos));  % 2D flat trajectory
% Update pitch and trajectory
pitch_angle(t+1) = pitch_angle(t) + x_sim(2,t)*Ts;
x_pos(t+1) = x_pos(t) + Vel*cos(pitch_angle(t+1))*Ts;
z_pos(t+1) = z_pos(t) + Vel*sin(pitch_angle(t+1))*Ts;

%% 3D Trajectory Visualization
time = 0:Ts:n_steps*Ts;
y_pos = zeros(size(x_pos));
figure;
plot3(x_pos/1000, y_pos/1000, z_pos/1000, 'b'); hold on;
plot3(x_pos(1)/1000, 0, z_pos(1)/1000, 'go', 'MarkerSize', 10);
plot3(x_pos(end)/1000, 0, z_pos(end)/1000, 'ro', 'MarkerSize', 10);
xlabel('x (km)'); ylabel('y (km)'); zlabel('z (km)');
title('3D Missile Trajectory - LQR'); grid on;


%% Animation 

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
    pause(0.5);
end
% -------------------- Performance Analysis --------------------
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
title('Control Input Distribution - LQR');
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
title('Az Tracking Error - LQR'); ylabel('Error (g)'); grid on;

subplot(2,1,2);
plot(t(1:end-1), tracking_error(2,:), 'b', 'LineWidth', 1.5);
title('Pitch Rate Tracking Error - LQR'); xlabel('Time (s)'); ylabel('Error'); grid on;
