% Jared Di Carlo, Patrick M. Wensing, Benjamin Katz, Gerardo Bledt, and Sangbae Kim
% 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)
% "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control"

clc;
clearvars -except time_history cost_history
close all;

% Default Parameters
params.m = 43;
params.g = -9.8;
params.I_body = diag([0.41, 2.1, 2.1]);
params.body_l = 0.73;
params.body_w = 0.24;
params.body_h = 0.24;
params.mu = 0.6;
params.control_dt = 0.025;
params.sim_dt = 0.001;
params.f_min = 10;
params.f_Max = 666;
params.global_time = 0;

% Gait Parameters
% 0: Standing on all
% 1: Trotting
% 2: Pronking/Jumping
% 3: Bounding
% 4: Galloping
params.gait = 0;
params.v_des = [0; 0; 0];  % m/s
params.a_des = 10;          % m/s^2 (0으로 두면 안됨 - Disturbance에 대응 불가)
params.w_des = 0;          % deg/s
params.alpha_des = 90;     % deg/s^2 (0으로 두면 안됨 - Disturbance에 대응 불가)

params.t_stance = 0.2;
params.t_swing = 0.1;

params.disturbance.time = [4; 4.3];
params.disturbance.F = [0; 0; 0];
params.disturbance.offset = [0; -0.12; 0.12]; % 0.365 0.12 0.12

params.k = 20;      % horizon length: 0.5s (= 0.25 * 20)

steps = 400;
sim_steps = round(params.control_dt / params.sim_dt);

Q = diag([ 1  1  1,  ... % roll, pitch, yaw weight
           0  0 50,  ... % x, y, z weight
           0  0  1,  ... % wx, wy, wz weight
           1  1  1,  ... % vx, vy, vz weight
           0]);

R_weight = 1e-6 * eye(12);

params.L = kron(eye(params.k), Q); 


% Initializion
Xc = zeros(13, 1);
Xc(1:13, 1) = [0; 0;   0;   % Euler's angles: roll, pitch, yaw
               0; 0; 0.6;   % Position: x, y, z
               0; 0;   0;   % Angular velocity
               0; 0;   0;   % Velocity
               params.g];   % Gravity acc. (cf. eq (16)-(17) from the main paper)

R_init = [ 0.365;  0.12; -0.6;
           0.365; -0.12; -0.6;
          -0.365;  0.12; -0.6;
          -0.365; -0.12; -0.6];

Xref = genRef(Xc, params);
contact = get_Contact(params);
R = get_R(Xc, Xref, contact, params, repmat(R_init, params.k, 1));

X_history = zeros(13, sim_steps * steps);
f_history = zeros(12, steps);
R_history = zeros(12, steps);
Xref_history = zeros(13*params.k, steps);
F_total_history = zeros(1, steps);
contact_history = zeros(1, steps);
time_history = zeros(2, steps);
cost_history = zeros(2, steps);

for step = 1:steps
    Xref = genRef(Xc, params);
    contact = get_Contact(params);
    R = get_R(Xc, Xref, contact, params, R);

    total_n = sum(sum(contact));
    params.K = eye(3 * total_n) * 1e-6;

    [H, g_qp] = get_QP(Xc, Xref, R, contact, params);
    H = (H+H')/2;
    
    [A_ineq, b_ineq, lb, ub] = get_Constraints(contact, params);

    tic
    [Uopt, cost] = quadprog(H, g_qp, A_ineq, b_ineq, [], [], lb, ub);
    time_history(2, step) = toc;
    cost_history(2, step) = cost;

    U_full_current = zeros(12, 1);
    current_contact = contact(:, 1);
    idx_uopt = 1;
    
    for j = 1:4
        if current_contact(j) == 1
            U_full_current(3*(j-1)+1 : 3*j) = Uopt(idx_uopt : idx_uopt+2);
            idx_uopt = idx_uopt + 3;
        end
    end
    
    [Xseries, params] = genSim(Xc, U_full_current, R, params);
    
    X_history(:, (step-1)*sim_steps + 1 : step*sim_steps) = Xseries; 
    R_history(:, step) = R(1:12, 1);
    f_history(:, step) = U_full_current;
    Xref_history(:, step) = Xref;
    F_total_history(:, step) = sum(U_full_current);
    contact_history(:, step) = sum(current_contact);
    
    Xc = Xseries(:, end);
    disp(step)
end

total_t = sum(time_history(2, :))
visualize(X_history, R_history, f_history, params);