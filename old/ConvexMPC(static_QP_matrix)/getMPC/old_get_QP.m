function [H, g] = old_get_QP(Xc, Xref, R, p)
% function to get QP formulation
% vector of all states during the horizon: eq (27)
% objective function: eq (28)
% standard form: eq (29)-(32)

% Xc: current state (13 x 1)
% Xref: reference over a horizon (13*p.k x 1)
% R: foot placement locations over a horizon (12*p.k x 1)
% p: parameter structure

Aqp = zeros(13*p.k, 13);
Bqp = zeros(13*p.k, 12*p.k);

Ac = get_A(Xref, p.k);
Ad = eye(13) + Ac * p.control_dt;


powerAd = Ad;

for i = 1:p.k
    row_idx = 13*(i-1) + 1 : 13*i;
    Aqp(row_idx, :) = powerAd;

    R_idx = R(12*(i-1) + 1 : 12*i, 1);

    Bc = old_get_B(Xref, R_idx, i, p);
    Bd = Bc * p.control_dt;

    col_idx = 12*(i-1) + 1 : 12*i;
    Bqp(row_idx, col_idx) = Bd;

    for j = i+1:p.k
        next_row_idx = 13*(j-1) + 1 : 13*j;
        prev_row_idx = 13*(j-2) + 1 : 13*(j-1);

        Bqp(next_row_idx, col_idx) = Ad * Bqp(prev_row_idx, col_idx);
    end

    powerAd = powerAd * Ad;
end

H = 2 * (Bqp' * p.L * Bqp + p.K);
g = 2 * Bqp' * p.L * (Aqp * Xc - Xref);

end