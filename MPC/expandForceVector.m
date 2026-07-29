function U_full = expandForceVector(U_active, current_contact)
%% Expand active-contact force variables into the fixed 12-by-1 leg force vector.
% U_active is ordered by active legs in the current horizon step.

current_contact = current_contact(:);
active_legs = find(current_contact);
required_count = 3 * numel(active_legs);

if numel(U_active) < required_count
    error('expandForceVector:InvalidSize', ...
        'Expected at least %d active force values, but got %d.', ...
        required_count, numel(U_active));
end

U_full = zeros(12, 1);

for active_idx = 1:numel(active_legs)
    leg = active_legs(active_idx);
    full_idx = 3*(leg-1) + 1 : 3*leg;
    active_force_idx = 3*(active_idx-1) + 1 : 3*active_idx;

    U_full(full_idx) = U_active(active_force_idx);
end
end
