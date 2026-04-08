function Xref = genRef(Xc, p)
% generate reference trajectory (Real-time Velocity Profiler 적용)
% Xc: current state [13x1]
% p: default parameters

Xref = zeros(13*p.k, 1);

Xc_base = Xc;
Xc_base([1, 2, 7, 8, 12], 1) = 0; % set roll, pitch, rollrate, pitchrate, Vz zero
Xc_base(6, 1) = 0.6;

v_target_base = p.v_des(1:2, 1);
a_des = p.a_des;
w_target = (pi / 180) * p.w_des;        % rad/s
alpha_des = (pi / 180) * p.alpha_des;  % rad/s^2
dt = p.control_dt;

is_w_zero = double(w_target ~= 0);

yaw_curr = Xc(3, 1);              % 현재 yaw
w_curr = Xc(9, 1);                % 현재 yaw rate
p_curr = Xc(4:5, 1);              % 현재 x, y
v_curr = Xc(10:11, 1);            % 현재 x, y rate


for i = 1:p.k
    index = 13*(i-1);
    Xref(index + 1 : index + 13) = Xc_base;

    % Rotation
    w_error = w_target - w_curr;
    alpha_req = w_error / dt;

    if abs(alpha_req) > abs(alpha_des)
        alpha_step = sign(alpha_req) * abs(alpha_des);
    else
        alpha_step = alpha_req;
    end

    yaw_next = yaw_curr + w_curr * dt + 0.5 * alpha_step * dt^2;
    w_next = w_curr + alpha_step * dt;

    Xref(index + 3) = yaw_next;
    Xref(index + 9) = w_next;

    % Transition
    v_target_world = Rz(yaw_curr * is_w_zero) * [v_target_base(1); v_target_base(2); 0];
    v_target_xy = v_target_world(1:2);

    v_error = v_target_xy - v_curr;
    v_err_norm = norm(v_error);

    if v_err_norm > 10^(-9)
        ax = v_error(1) / dt;
        ay = v_error(2) / dt;
    else
        ax = 0;
        ay = 0;
    end

    a_req = [ax; ay];
    a_req_norm = norm(a_req);

    if a_req_norm > a_des
        a_step = (a_des / a_req_norm) * a_req;
    else
        a_step = a_req;
    end


    pos_next = p_curr + v_curr * dt + 0.5 * a_step * dt^2;
    vel_next = v_curr + a_step * dt;

    Xref(index + 4 : index + 5) = pos_next;
    Xref(index + 10 : index + 11) = vel_next;


    yaw_curr = yaw_next;
    w_curr = w_next;
    p_curr = pos_next;
    v_curr = vel_next;
end

end