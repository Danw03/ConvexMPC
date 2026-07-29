% Jared Di Carlo, Patrick M. Wensing, Benjamin Katz, Gerardo Bledt, and Sangbae Kim
% 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)
% "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control"

% main.m 상단
% 루프 내부에서 필요한 데이터만 남기고 나머지(main 실행 결과물들) 싹 정리
clearvars -except rootPath oldPath nTrials nSteps trial raw_new_times raw_old_times;
clc;

% Default Parameters
params.m = 43;
params.g = -9.8;
params.I_body = diag([0.41, 2.1, 2.1]);
params.body_l = 0.73;
params.body_w = 0.24;
params.body_h = 0.24;
params.mu = 0.6; % Maximum static friction coefficient
params.control_dt = 0.025;       % Todo: 0.025
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
params.gait = 2;
params.v_des = [0; 0; 0];  % m/s
params.a_des = 10;          % m/s^2 (0으로 두면 안됨 - Disturbance에 대응 불가)
params.w_des = 0;          % deg/s
params.alpha_des = 90;     % deg/s^2 (0으로 두면 안됨 - Disturbance에 대응 불가)

params.t_stance = 0.3;
params.t_swing = 0.3;

params.disturbance.time = [4; 4.3];
params.disturbance.F = [0; 0; 0];
params.disturbance.offset = [0.365; -0.12; 0.12]; % 0.365 0.12 0.12

params.k = 10;      % horizon length: 0.5s (= 0.25 * 20)

steps = 400;
sim_steps = round(params.control_dt / params.sim_dt);

Q = diag([ 1  1  1,  ... % roll, pitch, yaw weight
           0  0 50,  ... % x, y, z weight
           0  0  1,  ... % wx, wy, wz weight
           1  1  1,  ... % vx, vy, vz weight
           0]);

R_weight = 1e-6 * eye(12);

params.L = kron(eye(params.k), Q); 
params.K = kron(eye(params.k), R_weight);


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
R = old_get_R(Xc, Xref, contact, params, repmat(R_init, params.k, 1));

X_history = zeros(13, sim_steps * steps);
f_history = zeros(12, steps);
R_history = zeros(12, steps);
Xref_history = zeros(13*params.k, steps);
F_total_history = zeros(1, steps);
contact_history = zeros(1, steps);
time_history = zeros(1, steps);
cost_history = zeros(1, steps);

for step = 1:steps
    Xref = genRef(Xc, params);
    contact = get_Contact(params);
    R = old_get_R(Xc, Xref, contact, params, R);
    
    [H, g_qp] = old_get_QP(Xc, Xref, R, params);
    H = (H+H')/2;
    [A_ineq, b_ineq, lb, ub] = get_Constraints(contact, params);

    % Solve the quadratic program
    tic
    [Uopt, cost] = quadprog(H, g_qp, A_ineq, b_ineq, [], [], lb, ub);
    time_history(1, step) = toc;
    cost_history(1, step) = cost;

    [Xseries, params] = genSim(Xc, Uopt, R, params);

    X_history(:, (step-1)*sim_steps + 1 : step*sim_steps) = Xseries; 
    f_history(:, step) = Uopt(1:12);
    R_history(:, step) = R(1:12);
    Xref_history(:, step) = Xref;
    F_total_history(:, step) = sum(Uopt(1:12), "all");
    contact_history(:, step) = sum(contact(:, 1), "all");

    Xc = Xseries(:, end);

    disp(step)
end

time_total = sum(time_history(1, :))
% visualize(X_history, R_history, f_history, params);