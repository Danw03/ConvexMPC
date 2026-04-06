function [A_ineq, b_ineq, lb, ub] = get_Constraints(contact, p)
% function to get QP constraints with time-varying contact

% contact: 4 x p.k matrix (1: Stance, 0: Swing)
% p: parameter structure

total_n = sum(sum(contact));

A_ineq = zeros(4*total_n, 3*total_n);
b_ineq = zeros(4*total_n, 1); 
lb = zeros(3*total_n, 1);
ub = zeros(3*total_n, 1);

A_foot = [ 1,  0, -p.mu;
          -1,  0, -p.mu;
           0,  1, -p.mu;
           0, -1, -p.mu ];

row_start = 1;
col_start = 1;

for i = 1:p.k
    for j = 1:4
        if contact(j, i) == 1
            row_idx = row_start : row_start + 3;
            col_idx = col_start : col_start + 2;

            A_ineq(row_idx, col_idx) = A_foot;
            lb(col_idx) = [-Inf; -Inf; p.f_min];
            ub(col_idx) = [ Inf;  Inf; p.f_Max];
            
            row_start = row_start + 4;
            col_start = col_start + 3;
        end

    end
end
end