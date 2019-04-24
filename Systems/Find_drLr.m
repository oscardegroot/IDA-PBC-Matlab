qdot = sym('qdot', [n_link, 1]);
q = sym('q', [n_link, 1]);
assume([q qdot], 'real');

Linv = inv(Psi'*Psi + epsilon*eye(2));
r = Psi'*qdot + lambda*z;
l = numel(r);

%% Synthesis using the sum derivation
rjL = [];
ziLzj = 0;
for i = 1 : l
    rjL_temp = 0;
    ziLzj_temp = 0;
    for j = 1 : l
        dLij_qdot = 0;
        for k = 1 : n_link
            % build the vector
            dLij_qdot = dLij_qdot + diff(Linv(i, j), q(k))*qdot(k);
        end
        rjL_temp = rjL_temp + r(j)*dLij_qdot;
        ziLzj_temp = ziLzj_temp + z(i)*dLij_qdot*z(j);
    end
    rjL = [rjL; rjL_temp];
    ziLzj = ziLzj + ziLzj_temp;
end
tau = rjL;

drLr = matlabFunction(tau);
drLr = @(q, qdot) drLr(q(1), q(2), q(3), qdot(1), qdot(2), qdot(3));
% dzLz = matlabFunction(ziLzj);
% dzLz = @(q, qdot) dzLz(q(1), q(2), q(3), qdot(1), qdot(2), qdot(3));