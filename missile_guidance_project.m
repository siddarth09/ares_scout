function missile_guidance(targets)
    % missile_guidance_project - Main function to run the entire simulation
    % Inputs:
    %   targets - A matrix of multiple targets [x, z] in meters

    % Initialize system
    [Ad, Bd, Cd, Dd, nx, nu, Qy, R, Vel, Ts, f] = initialize_system();

    % Initialize trajectory storage
    x_pos_total = [];
    z_pos_total = [];
    pitch_angle_total = [];
    u_sim_total = [];

    % Starting point
    x_start = 1590;
    z_start = 10000;

    % Define a moving window size (smaller than the entire trajectory)
    window_size = 5;

    % Iterate through the entire trajectory using a moving window approach
    for i = 1:window_size:length(targets)-window_size
        % Extract the current segment of the trajectory
        current_window = targets(i:i+window_size, :);

        % Run MPC on the current window
        [x_pos, z_pos, pitch_angle, u_sim] = run_mpc(current_window, Ad, Bd, Cd, Dd, nx, nu, Qy, R, Vel, Ts, f, x_start, z_start);

        % Append results after each segment
        x_pos_total = [x_pos_total, x_pos(2:end)];  
        z_pos_total = [z_pos_total, z_pos(2:end)];
        pitch_angle_total = [pitch_angle_total, pitch_angle(2:end)];
        u_sim_total = [u_sim_total, u_sim];

        % Update starting point for next segment
        x_start = x_pos(end);
        z_start = z_pos(end);
    end

    if length(pitch_angle_total) ~= length(x_pos_total)
        error('Mismatch: pitch_angle_total size (%d) does not match x_pos_total size (%d)', length(pitch_angle_total), length(x_pos_total));
    end

    % Plot the entire trajectory
    plot_trajectory(x_pos_total, z_pos_total, targets);
end

% Define start and end points in LLA (Latitude, Longitude, Altitude)
startLLA = [15, 15, 300];  % Initial point (Latitude, Longitude, Altitude in meters)
endLLA = [95, 95, 10000];    % Target point (Latitude, Longitude, Altitude in meters)

% Time of travel (in seconds)
timeOfTravel = [0, 3600];  % Assuming 1 hour travel time

% Sample rate (in Hz)
sampleRate = 1;  % 1 Hz sampling rate (adjust as needed)

% Generate trajectory using geoTrajectory
trajectory = geoTrajectory([startLLA; endLLA], timeOfTravel, 'SampleRate', sampleRate);

% Collect positions over time
positionsLLA = startLLA;
while ~isDone(trajectory)
    positionsLLA = [positionsLLA; trajectory()];  
end

% Define sample times (adjust for your simulation needs)
sampleTimes = 0:10:3600;  % Every 10 seconds

% Convert trajectory to ECEF (Earth-Centered, Earth-Fixed) coordinates
positionsCart = lookupPose(trajectory, sampleTimes, 'ECEF');

% Reference LLA point for ENU conversion (starting point)
refLLA = startLLA;  % Should be in [lat, lon, alt] format

% Convert ECEF positions to ENU (East-North-Up)
[xENU, yENU, zENU] = ecef2enu(positionsCart(:,1), positionsCart(:,2), positionsCart(:,3), ...
                              refLLA(1), refLLA(2), refLLA(3), wgs84Ellipsoid);

% Select only the x (East) and z (Up) components for your MPC
x_trajectory = xENU;  % East component (meters)
z_trajectory = zENU;  % Up component (altitude in meters)


n = 3;  % Adjust as needed for speed vs accuracy
x_trajectory = x_trajectory(1:n:end);
z_trajectory = z_trajectory(1:n:end);

% Combine into a 2D matrix for missile_guidance
targets = [x_trajectory, z_trajectory];  % Now in [x, z] format



% Run the missile guidance function with the generated targets
missile_guidance(targets);

