function contact = get_Contact(p)
%% Generate contact sequence over the MPC horizon.
% p: default parameters (contain gait, k, global_time)

contact = ones(4, p.k);
period = p.t_swing + p.t_stance;

switch p.gait
    case 0 % standing
        return
    case 1 % trotting: diagonal pairs
        contact = phaseContact(p, {[1, 4], [2, 3]}, [0, 0.5*period]);
    case 2 % pronking/jumping: all legs together
        contact = phaseContact(p, {1:4}, 0);
    case 3 % bounding: front pair, rear pair
        contact = phaseContact(p, {[1, 2], [3, 4]}, [0, 0.5*period]);
    case 4 % galloping: staggered front and rear legs
        delta_t = 0.5 * p.t_stance;
        contact = phaseContact(p, {1, 2, 3, 4}, ...
            [0, delta_t, 0.5*period, 0.5*period + delta_t]);
    case 5 % pacing: left pair, right pair
        contact = phaseContact(p, {[1, 3], [2, 4]}, [0, 0.5*period]);
end

end

function contact = phaseContact(p, leg_groups, phase_offsets)
period = p.t_swing + p.t_stance;
stance_time = round(p.t_stance, 4);
t = round(p.global_time, 4);
contact = zeros(4, p.k);

for step = 1:p.k
    for group_idx = 1:numel(leg_groups)
        phase = round(mod(t + phase_offsets(group_idx), period), 4);
        contact(leg_groups{group_idx}, step) = phase < stance_time;
    end

    t = round(t + p.control_dt, 4);
end
end
