q = sym('q', [3,1]);
assume(q, 'real')
syms e a g real
d = (1/e)^2 + (4*g/(a*e));
assume(d, 'positive');
l1 = (1/e + sqrt(d))/2;
dV = [0;0;-g/e*sin(q(3))];
Psi = [eye(2); [-1/l1*cos(q(3)) -1/l1*sin(q(3))]];
dVs = [0; 0; a*sin(q(3))];

Fp = [cos(q(3)) sin(q(3)) -e];
b = Fp*dV;

%a = 20;
%l1 = 1/e + sqrt((1/e)^2 - (4*g/(a*e)));

nPsi = [cos(q(3)) sin(q(3)) l1];

Fp*(dV - nPsi'*nPsi*dVs)