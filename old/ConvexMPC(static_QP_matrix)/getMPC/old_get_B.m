function B = old_get_B(Xref, r, horizon_idx, p)
% function to get time varying B matrix for one step in horizon
% governing equations: eq (5)-(7) from the main paper
% simplified continuous dynamics: eq (16)-(17) from the main paper
% Linear Discrete Time Varing Dynamics: eq (25)-(26) from the main paper

% Xref: reference over a horizon
% R_idx: foot placement locations (for one step)
% k: prediction horizon length
% p: default parameter structure

psi = Xref(13*(horizon_idx-1) + 3, 1);
I_world = Rz(psi) * p.I_body * Rz(psi)';

B = zeros(13, 12);

mass_inv = eye(3) / p.m;

for i = 1:4
    idx = 3*(i-1) + 1 : 3*i;

    B(7:9, idx) = I_world \ skewSymmetric(r(idx, 1));
    B(10:12, idx) = mass_inv;
end

end