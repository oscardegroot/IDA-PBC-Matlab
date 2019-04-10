% Calculates and saves the partial derivative of the inverse of the mass
% matrix to q


syms q3 real
k1 = 2;
k2 = 6.67;
k3 = 1.02;
e = 0.1;
Md = [k1*e*(cos(q3))^2 + k3 k1*e*cos(q3)*sin(q3) k1*cos(q3);...
                      k1*e*cos(q3)*sin(q3) -k1*e*(cos(q3))^2+k3 k1*sin(q3);...
                      k1*cos(q3) k1*sin(q3) k2];    

Md = inv(Md);
dMd_dq = matlabFunction(simplify(diff(Md, 'q3')));
dMd_dq = @(q, p) [0; 0; [p(1) p(2) p(3)] * dMd_dq(q(3)) * [p(1); p(2); p(3)]];

%save(filename, 'dMd_dq');