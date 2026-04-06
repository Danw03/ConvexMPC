function [H, g] = get_QP(Xc, Xref, R, contact, p)
% function to get QP formulation with time-varying contact
% Xc: current state (13 x 1)
% Xref: reference over a horizon (13*p.k x 1)
% R: foot placement locations over a horizon (12*p.k x 1)
% contact: 4 x p.k matrix (1: Stance, 0: Swing)
% p: parameter structure

total_n = sum(sum(contact));

Aqp = zeros(13*p.k, 13);
Bqp = zeros(13*p.k, 3*total_n);

Ac = get_A(Xref, p.k);
Ad = eye(13) + Ac * p.control_dt;
powerAd = Ad;

col_start = 1;

for i = 1:p.k
    row_idx = 13*(i-1) + 1 : 13*i;
    
    Aqp(row_idx, :) = powerAd;
    
    n_i = sum(contact(:, i));
    num_cols_i = 3 * n_i;

    if num_cols_i > 0
        R_idx = R(12*(i-1) + 1 : 12*i, 1);

        Bc = get_B(Xref, R_idx, i, contact, p);
        Bd = Bc * p.control_dt;

        col_idx = col_start : (col_start + num_cols_i - 1); 
        
        Bqp(row_idx, col_idx) = Bd;

        for j = i+1:p.k
            next_row_idx = 13*(j-1) + 1 : 13*j;
            prev_row_idx = 13*(j-2) + 1 : 13*(j-1);
            Bqp(next_row_idx, col_idx) = Ad * Bqp(prev_row_idx, col_idx);
        end

        col_start = col_start + num_cols_i;
    end

    powerAd = powerAd * Ad;
end

H = 2 * (Bqp' * p.L * Bqp + p.K);
g = 2 * Bqp' * p.L * (Aqp * Xc - Xref);

end