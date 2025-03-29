function plot_trajectory(x_pos_total, z_pos_total, pitch_angle_total)
    % Animate the missile's trajectory in 3D

    % Missile body dimensions (in km)
    missile_length = 0.1; % 100 meters
    missile_radius = 0.02; % 20 meters

    % Create cone-shaped missile geometry
    [cone_x, cone_y, cone_z] = cylinder([0 missile_radius], 20);
    cone_z = missile_length * cone_z;

    % Generate a y_pos vector for 2D plane (all zeros)
    y_pos_total = zeros(1, length(x_pos_total));

    % Create figure
    figure;
    hold on;
    axis equal;
    grid on;
    xlabel('x (km)'); ylabel('y (km)'); zlabel('z (km)');
    title('Missile Trajectory Animation');
    view(3);

    % Plot start and end points
    plot3(x_pos_total(1)/1000, y_pos_total(1)/1000, z_pos_total(1)/1000, 'go', 'MarkerSize', 8, 'DisplayName', 'Start');
    plot3(x_pos_total(end)/1000, y_pos_total(end)/1000, z_pos_total(end)/1000, 'ro', 'MarkerSize', 8, 'DisplayName', 'End');

    % Set up missile surface (initially hidden)
    missile_surf = surf(nan(size(cone_x)), nan(size(cone_y)), nan(size(cone_z)), ...
        'FaceColor', 'r', 'EdgeColor', 'none');

    % Plot initial trail
    h = plot3(nan, nan, nan, 'b', 'LineWidth', 1.5);  % Initialize empty plot for trail

    % Plot cone every 'step_size' frames for smoother animation
    step_size = 5;

    % Loop through trajectory
    for k = 2:step_size:min(length(x_pos_total), length(pitch_angle_total))
        % Missile center position (in km)
        center = [x_pos_total(k), y_pos_total(k), z_pos_total(k)] / 1000;

        % Get current pitch angle
        theta = pitch_angle_total(k);  % already in radians

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

        % Update trail dynamically during animation
        set(h, 'XData', x_pos_total(1:k)/1000, ...
               'YData', y_pos_total(1:k)/1000, ...
               'ZData', z_pos_total(1:k)/1000);

        drawnow limitrate;  % Use 'limitrate' for smoother animation control
        pause(0.05);  % Control speed of animation
    end

    % Plot final trajectory for clarity after animation
    plot3(x_pos_total/1000, y_pos_total/1000, z_pos_total/1000, 'b', 'LineWidth', 1.5); 

    % Plot formatting
    legend('Start', 'End', 'Trajectory');
end
