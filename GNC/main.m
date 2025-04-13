% MAIN_COMPARE_CONTROLLERS.M
% Runs all missile controllers and visualizes their 3D flight trajectories

clear; clc; close all;

% Run all controllers
metrics_lqr = run_missile_controller('LQR');
metrics_mpc = run_missile_controller('MPC');
metrics_sfi = run_missile_controller('SFI');
controllers = {metrics_lqr, metrics_mpc, metrics_sfi};

colors = {'r', 'b', 'g'};  % LQR (Red), MPC (Blue), SFI (Green)
labels = {'LQR + Integral', 'MPC', 'SFI'};


cone_radius = 0.02; cone_length = 0.1;

% Create figure
figure;
hold on; grid on; axis equal;
xlabel('x (km)'); ylabel('y (km)'); zlabel('z (km)');
title('Missile 3D Trajectories - MPC vs LQR vs SFI');
view(3);

% Draw ground target
plot3(10.5, 0, 3.0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');  % Target

legend_entries = cell(1, numel(controllers));

% Generate cone geometry
[cone_x, cone_y, cone_z] = cylinder([0 cone_radius], 20);
cone_z = cone_length * cone_z;

% Initialize plots
for i = 1:numel(controllers)
    c = controllers{i};
    trail{i} = plot3(nan, nan, nan, colors{i}, 'LineWidth', 1.5);
    cones{i} = surf(nan(size(cone_x)), nan(size(cone_y)), nan(size(cone_z)), ...
                    'FaceColor', colors{i}, 'EdgeColor', 'none');
    legend_entries{i} = labels{i};
end

legend(legend_entries, 'Location', 'best');

% Unified animation loop (assume equal time steps)
n_frames = length(controllers{1}.x_pos);
for k = 1:n_frames
    for i = 1:numel(controllers)
        x = controllers{i}.x_pos(k) / 1000;
        y = 0;
        z = controllers{i}.z_pos(k) / 1000;
        theta = controllers{i}.pitch_angle(k);
        
        % Update trail
        set(trail{i}, 'XData', controllers{i}.x_pos(1:k)/1000, ...
                      'YData', zeros(1,k), ...
                      'ZData', controllers{i}.z_pos(1:k)/1000);

        % Update cone orientation
        R = [cos(theta) 0 sin(theta);
             0 1 0;
             -sin(theta) 0 cos(theta)];
        rotated = R * [cone_x(:)'; cone_y(:)'; cone_z(:)'];
        Xr = reshape(rotated(1,:) + x, size(cone_x));
        Yr = reshape(rotated(2,:) + y, size(cone_y));
        Zr = reshape(rotated(3,:) + z, size(cone_z));
        set(cones{i}, 'XData', Xr, 'YData', Yr, 'ZData', Zr);
    end
    drawnow;
    pause(0.05);
end
