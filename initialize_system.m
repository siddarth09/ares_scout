function [Ad, Bd, Cd, Dd, nx, nu, Qy, R, Vel, Ts, f] = initialize_system()
    % initialize_system - Initializes the plant, discretization, and CasADi model

    import casadi.*
    
    %% Missile Longitudinal Dynamics
    A = [-1.064 1.000; 290.26 0.00];
    B = [-0.25; -331.40];
    C = [-123.34 0.00; 0.00 1.00];
    D = [0;0];

    %% Discretization
    Ts = 0.1;
    sysd = c2d(ss(A,B,C,D), Ts);
    Ad = sysd.A; Bd = sysd.B; Cd = sysd.C; Dd = sysd.D;

    nx = size(Ad,1); nu = size(Bd,2);
    Qy = diag([2, 0.5]); R = 1;

    %% CasADi symbolic model
    x = MX.sym('x',nx); u = MX.sym('u',nu);
    x_next = Ad*x + Bd*u;
    f = Function('f', {x, u}, {x_next});

    %% Missile parameters
    Vel = 1021.08;  % m/s constant forward velocity
end
