% Jared Di Carlo, Patrick M. Wensing, Benjamin Katz, Gerardo Bledt, and Sangbae Kim
% 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)
% "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control"

clc;
clear;
close all;

project_root = fileparts(mfilename('fullpath'));
addpath(fullfile(project_root, 'genRef'), ...
        fullfile(project_root, 'MPC'), ...
        fullfile(project_root, 'sim'), ...
        fullfile(project_root, 'utils'));

%% Default Parameters
params.m = 43;
params.g = -9.8;
params.I_body = diag([0.41, 2.1, 2.1]);
params.body_l = 0.73;
params.body_w = 0.24;
params.body_h = 0.24;
params.mu = 0.6;
params.f_min = 10;
params.f_Max = 666;
params.global_time = 0;
params.sim_dt = 0.001;

%% MPC Parameters
params.k = 20;      % horizon length: 0.5s (= 0.025 * 20)
params.control_dt = 0.025;

%% Gait Parameters
% 0: Standing on all
% 1: Trotting
% 2: Pronking/Jumping
% 3: Bounding
% 4: Galloping
% 5: Pacing
params.gait = 1;
params.v_des = [1; 0; 0];  % m/s
params.a_des = 10;         % m/s^2
params.w_des = 0;          % deg/s
params.alpha_des = 30;     % deg/s^2

params.t_stance = 0.1;
params.t_swing = 0.2;

%% Disturbance Parameters
params.disturbance.time = [4; 4.2];
params.disturbance.F = [0; 0; 0];
params.disturbance.offset = [0; -0.12; 0.]; % 0.365 0.12 0.12

%% Weight Parameters
Q = diag([ 1  1  1,  ... % roll, pitch, yaw weight
           0  0 50,  ... % x, y, z weight
           0  0  1,  ... % wx, wy, wz weight
           1  1  1,  ... % vx, vy, vz weight
           0]);

R_weight = 1e-6 * eye(12);

params.L = kron(eye(params.k), Q); 
params.K = kron(eye(params.k), R_weight);

%% Simulation Configuration
steps = 400;    % simulation time: 10s (0.025 * 400 = 10)
sim_steps = round(params.control_dt / params.sim_dt);


%% Initializion
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


%% History Structure
history.X = zeros(13, sim_steps * steps);
history.F = zeros(12, steps);
history.R = zeros(12, steps);
history.Xref = zeros(13 * params.k, steps);
history.F_total = zeros(1, steps);
history.contact = zeros(1, steps);
history.solver_time = zeros(1, steps);
history.solver_cost = zeros(1, steps);

%% Main Loop
for step = 1:steps
    Xref = genRef(Xc, params);
    contact = get_Contact(params);
    R = get_R(Xc, Xref, contact, params, R);

    active_force_count = 3 * nnz(contact);
    params.K = eye(active_force_count) * 1e-6;

    [H, g_qp] = get_QP(Xc, Xref, R, contact, params);
    H = (H+H')/2;
    
    [A_ineq, b_ineq, lb, ub] = get_Constraints(contact, params);

    tic
    [Uopt, cost] = quadprog(H, g_qp, A_ineq, b_ineq, [], [], lb, ub);
    history.solver_time(1, step) = toc;
    history.solver_cost(1, step) = cost;

    current_contact = contact(:, 1);
    U_full_current = expandForceVector(Uopt, current_contact);
    
    [Xseries, R, params] = genSim(Xc, U_full_current, R, contact, params);

    history = genHistory(history, step, sim_steps, Xseries, R, U_full_current, Xref, current_contact);

    Xc = Xseries(:, end);

    fprintf("step %d / %d:\n  cost = %.6f\n\n", step, steps, cost);
end

finalReport(history);
visualize(history, params);
