clc; clear;

%% Time Setup
Ts = 0.1;
t_final = 80;
t = 0:Ts:t_final;
N = length(t);

%% Missile Dynamics
A = [-0.0089  -0.1474     0   -9.75;
     -0.0216  -0.3601   5.947  -0.151;
      0      -0.00015 -0.0224  0.0006;
      0       1         0       0];
B = [0; 9.748; 3.77; -0.034];
C = eye(4); D = zeros(4,1);
plant = ss(A, B, C, D);

%% MPC Setup
mpcobj = mpc(plant, Ts, 50, 10);
mpcobj.Weights.OutputVariables = [0, 1, 0, 0.5];
mpcobj.Weights.ManipulatedVariables = 0.01;
mpcobj.Weights.ManipulatedVariablesRate = 0.1;

mpcobj.MV.Min = deg2rad(-20); mpcobj.MV.Max = deg2rad(20);
rate_limit = deg2rad(60);
mpcobj.MV.RateMin = -rate_limit * Ts;
mpcobj.MV.RateMax = rate_limit * Ts;
mpcDesigner(mpcobj)



