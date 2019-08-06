% q = sym('q', [3, 1]);

syms b1 b2 b3 real

% assume([q], 'real');
% Fp = System.Fp(q);
% Psi = System.Psi(q);
syms l real

out = solve([-(l)*b1 + b3 + l*e*b2 == 0, b1 > 0, b2 < b1/e, b2 > b1/(2*e), b3 > 5*e*b1], [b1 b2 b3 l], 'ReturnConditions', true);
out.conditions
b = [out.b1; out.b2; out.b3];
b = matlabFunction(b)

% Md = [out.Md1_1, out.Md1_2, out.Md1_3;...
%       out.Md2_1, out.Md2_2, out.Md2_3;...
%       out.Md3_1, out.Md3_2, out.Md3_3]
  
% Md = matlabFunction(Md)