function A = get_A(Xref, k)
% function to get avg A matrix over a horizon
% governing equations: eq (5)-(7) from the main paper
% simplified continuous dynamics: eq (16)-(17) from the main paper
% Linear Discrete Time Varing Dynamics: eq (25)-(26) from the main paper

% Xref: reference over a horizon
% k: prediction horizon length

A = zeros(13, 13);
A(4:6, 10:12) = eye(3);
A(12, 13) = 1;

yawAvg = mean(Xref(3 : 13 : 13*k));
A(1:3, 7:9) = Rz(yawAvg)';

end