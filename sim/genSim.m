function [Xseries, p] = genSim(Xc, Uopt, r, p)
% function to generate Sim.

% Xc: current state (13 x 1)
% Uopt: Solution from QP solver (12*k x 1)
% r: current footstep location (12 x 1)
% p: parameter structure

f_opt = Uopt(1:12, 1);
steps = round(p.control_dt / p.sim_dt);
Xseries = zeros(13, steps);

for i = 1:steps
    Xdot = simDynamics(Xc, f_opt, r, p);
    Xc = Xc + Xdot * p.sim_dt;
    Xseries(:, i) = Xc;
    p.global_time = p.global_time + p.sim_dt;
end

end