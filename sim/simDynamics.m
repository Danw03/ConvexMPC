function Xdot = simDynamics(Xc, u, r, p)
% function to get Dynamics matrix for Sim (non-linear)

[F_dist, tau_dist] = genDisturbance(p);
Xdot = rigidBodyDerivative(Xc, u, r, p, F_dist, tau_dist);
end
