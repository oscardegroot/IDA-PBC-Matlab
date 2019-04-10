qdot = sym('qdot', [n_link, 1]);
q = sym('q', [n_link, 1]);
assume([q qdot], 'real');

Linv = inv(Psi'*Psi + epsilon*eye(2));
r = Psi'*Mm'*qdot + lambda*z;
l = numel(r);
%% Synthesis using the sum derivation
% vdot = 0;
% for i = 1 : l
%     for j = 1 : l
%         dLij_qdot = 0;
%         for k = 1 : n_link
%             % build the vector
%             dLij_qdot = dLij_qdot + diff(Linv(i, j), q(k))*qdot(k);
%         end
%         vdot = vdot + r(i)*r(j)*dLij_qdot;
%     end
% end
% vdot
rjL = [];
for i = 1 : l
    rjL_temp = 0;
    for j = 1 : l
        dLij_qdot = 0;
        for k = 1 : n_link
            % build the vector
            dLij_qdot = dLij_qdot + diff(Linv(i, j), q(k))*qdot(k);
        end
        rjL_temp = rjL_temp + r(j)*dLij_qdot;
    end
    rjL = [rjL; rjL_temp];
end
tau = rjL;

% %% Derivation for rdLr and drLdr
% drdq = dAdq(r, n_link);
% 
% drLdr = (drdq'*Linv);%(dPsidq' + lambda*Psi + dMqdotdq'*Psi)*Linv;
% 
% % Multiply the first rank
% rdLr = [];%zeros(n_link, 1);
% for i = 1 : n_link
%     rdLr = [rdLr; 0.5*r'*diff(Linv, q(i))];
% end
% %rdLr = simplify(rdLr);
% 
% % Basically damping, could also be in Kv!
% drLr = (rdLr + drLdr)'*qdot;
% %
drLr = matlabFunction(tau);
drLr = @(q, qdot) drLr(q(1), q(2), q(3), qdot(1), qdot(2), qdot(3));