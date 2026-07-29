function [Xseries, R, p] = genSim(Xc, Uopt, R, contact, p)
r = R(1:12, 1);
current_contact = contact(1:4, 1);
f_opt = Uopt(1:12, 1);

steps = round(p.control_dt / p.sim_dt);
Xseries = zeros(13, steps);

for i = 1:steps
    Xdot = simDynamics(Xc, f_opt, r, p);
    Xc = Xc + Xdot * p.sim_dt;
    p.global_time = p.global_time + p.sim_dt;

    omega = Xc(7:9, 1);  
    v = Xc(10:12, 1);
    for leg = 1:4
        if current_contact(leg) == 1
            idx = 3*(leg-1)+1 : 3*leg;
            r(idx, 1) = r(idx, 1) - (v + cross(omega, r(idx, 1))) * p.sim_dt;
        end
    end

    Xseries(:, i) = Xc;
end

for leg = 1:4
    if current_contact(leg) == 1
        idx = 3*(leg-1)+1 : 3*leg;
        R(idx, 1) = r(idx, 1);
    end
end
end
