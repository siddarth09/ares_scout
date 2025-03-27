clc; clear;

%% Time Setup
Ts = 0.02;
t_final = 80;
t = 0:Ts:t_final;
N = length(t);

%% State-Space Model (From Image)
A = [-0.0089  -0.1474     0   -9.75;
     -0.0216  -0.3601   5.947  -0.151;
      0      -0.00015 -0.0224  0.0006;
      0       1         0       0];

B = [0;
     9.748;
     3.77;
    -0.034];

C = eye(4);
D = zeros(4,1);

%% Initial & Target Geodetic Position
LAT_INIT = 34.2329;   LON_INIT = -119.4573;   ELEV_INIT = 10000;
LAT_TARGET = 34.6588; LON_TARGET = -118.7697; ELEV_TARGET = 795;

target_vec = [LAT_TARGET - LAT_INIT;
              LON_TARGET - LON_INIT;
              ELEV_TARGET - ELEV_INIT];

range_m = 1021.08 * t_final; % ~linear range in meters

%% Simulation Setup
x = zeros(4, N);      % [u; w; q; theta]
x(:,1) = [1021.08; 0; 0; 0];  % Initial horizontal velocity, rest 0

delta_p = zeros(1,N);  % Fin deflection input

% Missile position over time
lat = zeros(N,1); lon = zeros(N,1); alt = zeros(N,1);
lat(1) = LAT_INIT; lon(1) = LON_INIT; alt(1) = ELEV_INIT;

% Convert to meters (approx)
dlat_per_m = (LAT_TARGET - LAT_INIT) / range_m;
dlon_per_m = (LON_TARGET - LON_INIT) / range_m;
dalt_per_m = (ELEV_TARGET - ELEV_INIT) / range_m;

%% Simple PD Controller to track downward flight angle
Kp = 0.01;
Kd = 0.2;

for k = 1:N-1
    % Simple control on theta to dive
    err_theta = deg2rad(-45) - x(4,k);  % desired pitch angle
    delta_p(k) = Kp * err_theta - Kd * x(3,k);
    delta_p(k) = max(min(delta_p(k), deg2rad(20)), deg2rad(-20));

    % State update (Euler integration)
    dx = A * x(:,k) + B * delta_p(k);
    x(:,k+1) = x(:,k) + Ts * dx;

    % Approximate position update
    u = x(1,k); w = x(2,k);
    dx_pos = u * Ts;
    dz_pos = w * Ts;

    lat(k+1) = lat(k) + dlat_per_m * dx_pos;
    lon(k+1) = lon(k) + dlon_per_m * dx_pos;
    alt(k+1) = alt(k) + dalt_per_m * dz_pos;
end

%% 3D Animation
figure;
hold on; grid on;
xlabel('Latitude'); ylabel('Longitude'); zlabel('Altitude (m)');
title('Missile Longitudinal Flight (Linearized Model)');
view(3);

% Plot target
plot3(LAT_TARGET, LON_TARGET, ELEV_TARGET, 'ro', 'MarkerSize', 8, 'DisplayName', 'Target');

% Live plot handles
hPath = plot3(NaN, NaN, NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Missile Path');
hMissile = plot3(NaN, NaN, NaN, 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'Missile');

legend;

for k = 2:5:N
    set(hPath, 'XData', lat(1:k), 'YData', lon(1:k), 'ZData', alt(1:k));
    set(hMissile, 'XData', lat(k), 'YData', lon(k), 'ZData', alt(k));
    drawnow;
end
