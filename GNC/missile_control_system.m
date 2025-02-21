function [A_close,B_close,C,D,new_sys] = missile_control_system(Vm0, m, IYY, x_bar, AX0, Cz_alpha0, Cm_alpha0, Cz_delta_p0, Cm_delta_p0, Q_bar, S, d, g)
    % Compute State-Space Model
    [A, B, C, D, sys] = missile_state_space(Vm0, m, IYY, x_bar, AX0, Cz_alpha0, Cm_alpha0, Cz_delta_p0, Cm_delta_p0, Q_bar, S, d, g);

    % Compute Transfer Function
    trans_func = tf(sys);

    % Controllability Analysis
    P = ctrb(A, B);
    if rank(P) == size(A, 1)
        disp("The system is controllable");
    else
        disp("The system cannot be controlled");
    end

    % Observability Analysis
    Q_obsv = obsv(A, C);
    if rank(Q_obsv) == size(A, 1)
        disp("The system is observable");
    else
        disp("The system cannot be observed");
    end

    % Eigenvalues for Stability Analysis
    [T, lambda] = eig(A);

    % LQR Control Design
    Q = [0.5 0; 0 0.5];
    R = 0.5;
    [K, S, E] = lqr(sys, Q, R);

    % Closed-loop System
    A_close = A - B * K;
    B_close = B;
    new_sys = ss(A_close, B_close, C, D);

    % Display Eigenvalues of A - B*K
    disp("Eigenvalues of A - B*K:");
    disp(eig(A_close));
end
