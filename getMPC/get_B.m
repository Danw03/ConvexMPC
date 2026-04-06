function B = get_B(Xref, r, horizon_idx, contact, p)
% function to get time varying B matrix for one step in horizon
% governing equations: eq (5)-(7) from the main paper
% simplified continuous dynamics: eq (16)-(17) from the main paper
% Linear Discrete Time Varing Dynamics: eq (25)-(26) from the main paper

% Xref: reference over a horizon
% r: foot placement locations (for one step)
% horizon_idx: current step index in prediction horizon
% contact: 4 x p.k matrix (1: Stance, 0: Swing)
% p: default parameter structure

psi = Xref(13*(horizon_idx-1) + 3, 1);
I_world = Rz(psi) * p.I_body * Rz(psi)';
mass_inv = eye(3) / p.m;

current_contact = contact(:, horizon_idx);
n_active = sum(current_contact);

B = zeros(13, 3 * n_active);

active_col_idx = 1; 

for i = 1:4
    if current_contact(i) == 1
        r_idx = 3*(i-1) + 1 : 3*i; 

        B_col_range = 3*(active_col_idx-1) + 1 : 3*active_col_idx;

        B(7:9, B_col_range) = I_world \ skewSymmetric(r(r_idx, 1));
        B(10:12, B_col_range) = mass_inv;
        active_col_idx = active_col_idx + 1;
    end
end

end