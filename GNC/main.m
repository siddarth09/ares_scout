%% Define missile parameters
Vm0 = 3350;     % ft/sec
m = 11.1;       % slug
IYY = 137.8;    % slug-ft^2
x_bar = 1.2;    % ft
AX0 = -60;      % ft/sec^2
Cz_alpha0 = -5.5313;
Cm_alpha0 = 6.6013; % Change to -6.6013 for stability
Cz_delta_p0 = -1.2713;
Cm_delta_p0 = -7.5368;
Q_bar = 13332;  % lb/ft^2
S = 0.5454;     % ft^2
d = 0.8333;     % ft
g = 32.174;     % ft/sec^2


%% Location parameters 

LAT_target=42.3400110308325;
LON_target=-71.08988920406841;
ELEV_target=795;

LAT_initial=42.36576446083541;
LON_initial=-70.4879934617387;
ELEV_initial=10000;

% Earth param

R= 6371e3;
d2r=pi/180;


%% Call the function
[A, B, C, D, sys] = missile_state_space(Vm0, m, IYY, x_bar, AX0, Cz_alpha0, Cm_alpha0, Cz_delta_p0, Cm_delta_p0, Q_bar, S, d, g);
[A_cl,B_cl,C,D,sys_cl] = missile_control_system(Vm0, m, IYY, x_bar, AX0, Cz_alpha0, Cm_alpha0, Cz_delta_p0, Cm_delta_p0, Q_bar, S, d, g);

trans_func=tf(sys)
cl_trans_func=tf(sys_cl)

%% LQG control with state estimation using kalman filter 

% define noise

G=eye(2);
H=0*eye(2);

Q_bar=diag(0.00015*ones(1,2))
R_bar=diag(0.55*ones(1,2))

% define noisy system

sys_estimated=ss(A_cl,[B_cl,G],C,[D,H])
[K_est,L,P]=kalman(sys_estimated,Q_bar,R_bar)


%% Observer closed loop 

A_ob=A_cl-L*C
fprintf("Observer eigen values \n")
disp(eig(A_ob))

dt1=0.75
dt2=0.25 


l1=LAT_initial*d2r;
l2=LAT_target*d2r;
u1=LON_initial*d2r;
u2=LON_target*d2r;

dl=l2-l1;
du=u2-u1;

a = sin(dl/2)^2 + cos(l1)*cos(l2)*sin(du/2)^2;

c = 2*atan2(sqrt(a),sqrt(1-a));

d = R*c; %horizontal distance (in m)

% initial range

r= sqrt(d^2+(ELEV_target-ELEV_initial)^2);

%% azimuth calculator

yaw_initial=azimuth(LAT_initial,LON_initial,LAT_target,LON_target)
yaw=yaw_initial*d2r

% initial flight path angle

dh=abs(ELEV_target-ELEV_initial);
FPA_INIT=atan(dh/d) %rad



