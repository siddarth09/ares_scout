function [A_num, B_num, C_num, D_num, sys] = missile_state_space(Vm0, m, IYY, x_bar, AX0, Cz_alpha0, Cm_alpha0, Cz_delta_p0, Cm_delta_p0, Q_bar, S, d, g)
    % Function to compute state-space representation of missile dynamics
    % Inputs:
    % Vm0         : Total Missile Velocity (ft/sec)
    % m           : Total Missile Mass (slug)
    % IYY         : Pitch Moment of Inertia (slug-ft^2)
    % x_bar       : Distance from CG to IMU (ft)
    % AX0         : Axial Acceleration (ft/sec^2)
    % Cz_alpha0   : Pitch Force Coefficient due to Angle of Attack
    % Cm_alpha0   : Pitch Moment Coefficient due to Angle of Attack
    % Cz_delta_p0 : Pitch Force Coefficient due to Fin Deflection
    % Cm_delta_p0 : Pitch Moment Coefficient due to Fin Deflection
    % Q_bar       : Dynamic Pressure (lb/ft^2)
    % S           : Reference Area (ft^2)
    % d           : Reference Length (ft)
    % g           : Gravity Constant (ft/sec^2)

    % State-space matrices (exact from the given image)
    A_num = [ (1/Vm0) * ( (Q_bar * S * Cz_alpha0) / m - AX0),  1;
              (Q_bar * S * d * Cm_alpha0) / IYY,              0];

    B_num = [ (Q_bar * S * Cz_delta_p0) / (m * Vm0);
              (Q_bar * S * d * Cm_delta_p0) / IYY];

    C_num = [ (Q_bar * S * Cz_alpha0) / (m * g) - (Q_bar * S * d * Cm_alpha0 * x_bar) / (g * IYY)  0;
              0, 1];

    D_num = [ (Q_bar * S * Cz_delta_p0) / (m * g) - (Q_bar * S * d * Cm_delta_p0 * x_bar) / (g * IYY);
              0];

    % Display numerical state-space matrices
    disp('State-Space Matrices (Corrected):');
    disp('A = '); disp(A_num);
    disp('B = '); disp(B_num);
    disp('C = '); disp(C_num);
    disp('D = '); disp(D_num);

    % Create state-space system
    sys = ss(A_num, B_num, C_num, D_num);

    
end
