q = sym('q', [3, 1]); %Lv = sym('l', [3, 3]); 
L = sym('l', [3, 1]);
assume([q L], 'real');

syms L1(q3) L2(q3) L3(q3) real

Md = System.Md(q);
Mdinv = simplify(inv(Md));
F = System.F(q);
M = System.M(q);

I_F = F*inv(F'*F)*F';
part = Mdinv*I_F*Md;
Lv = diag([L1; L2(q3); L3(q3)]);
Lv_ans = solve([det(part*Lv) >= 0, det(Lv) > 0], [L1]);

fprintf('hi');