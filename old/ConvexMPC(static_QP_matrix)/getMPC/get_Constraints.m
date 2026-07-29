function [A_ineq, b_ineq, lb, ub] = get_Constraints(contact, p)
% function to get QP constraints

% contact: 4 x p.k matrix (1: Stance, 0: Swing)
% p: parameter structure

A_ineq = zeros(16*p.k, 12*p.k);
b_ineq = zeros(16*p.k, 1);
lb = zeros(12*p.k, 1);
ub = zeros(12*p.k, 1);

A_foot = [ 1,  0, -p.mu;
          -1,  0, -p.mu;
           0,  1, -p.mu;
           0, -1, -p.mu ];

for i = 1:p.k
    for j = 1:4
        u_idx = 12*(i-1) + 3*(j-1) + 1 : 12*(i-1) + 3*j;

        c_idx = 16*(i-1) + 4*(j-1) + 1 : 16*(i-1) + 4*j;

        if contact(j, i) == 1
            A_ineq(c_idx, u_idx) = A_foot;
            lb(u_idx) = [-Inf; -Inf; p.f_min];
            ub(u_idx) = [ Inf;  Inf; p.f_Max];
        else
            lb(u_idx) = [0; 0; 0];
            ub(u_idx) = [0; 0; 0];
        end
    end
end
end