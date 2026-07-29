function [A_ineq, b_ineq, lb, ub] = get_Constraints(contact, p)
%% function to get QP constraints with time-varying contact

% contact: 4 x p.k matrix (1: Stance, 0: Swing)
% p: parameter structure

total_n = nnz(contact);

A_ineq = zeros(4*total_n, 3*total_n);
b_ineq = zeros(4*total_n, 1); 
lb = zeros(3*total_n, 1);
ub = zeros(3*total_n, 1);

A_foot = [ 1,  0, -p.mu;
          -1,  0, -p.mu;
           0,  1, -p.mu;
           0, -1, -p.mu ];

active_idx = 0;

for horizon_idx = 1:p.k
    active_legs = find(contact(:, horizon_idx));

    for leg_idx = 1:numel(active_legs)
        active_idx = active_idx + 1;
        row_idx = 4*(active_idx-1) + 1 : 4*active_idx;
        col_idx = 3*(active_idx-1) + 1 : 3*active_idx;

        A_ineq(row_idx, col_idx) = A_foot;
        lb(col_idx) = [-Inf; -Inf; p.f_min];
        ub(col_idx) = [ Inf;  Inf; p.f_Max];
    end
end
end
