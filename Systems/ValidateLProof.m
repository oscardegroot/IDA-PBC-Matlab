% Validate that dL/dq*q_dot is antisymmetric such that S = 0 and the system
% is lossless in terms of r-passivity

L = inv(System.Psi(q)'*System.Psi(q));

dLdq1 = diff(L, 'q1');
dLdq2 = diff(L, 'q2');
dLdq3 = diff(L, 'q3');

dLdq1-dLdq1'
dLdq2-dLdq2'
dLdq3-dLdq3'

dLdq(1, :, :) = dLdq1;
dLdq(2, :, :) = dLdq2;
dLdq(3, :, :) = dLdq3;

dLdq = matlabFunction(dLdq);
dLdq = @(q) dLdq(q(1), q(2), q(3));

q = [2; 2; 3];

dLdq(q) - permute(dLdq(q), [2 1 3])







