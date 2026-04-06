function Xdot = simDynamics(Xc, u, r, p)
% function to get Dynamics matrix for Sim (non-linear)

phi = Xc(1, 1);
theta = Xc(2, 1);
psi = Xc(3, 1);
J = jacobian(theta, psi);
[F_dist, tau_dist] = genDisturbance(p);

omega = Xc(7:9, 1);
v = Xc(10:12, 1);
m = p.m;
g_vec = [0; 0; p.g];

R = Rz(psi) * Ry(theta) * Rx(phi);
I_world = R * p.I_body * R';

sum_f = zeros(3, 1);
sum_tau = zeros(3, 1);

for i = 1:4
    index = 3*(i-1)+1 : 3*i;
    f_i = u(index, 1);
    r_i = r(index, 1);
    sum_f = sum_f + f_i;
    sum_tau = sum_tau + skewSymmetric(r_i) * f_i;
end

Theta_dot = J \ omega;  

p_dot = v;
coriolis  = skewSymmetric(omega) * (I_world * omega);
omega_dot = I_world \ (sum_tau + tau_dist - coriolis);
v_dot = (sum_f / m) + g_vec + F_dist / m;
Xdot = [Theta_dot; p_dot; omega_dot; v_dot; 0];
end