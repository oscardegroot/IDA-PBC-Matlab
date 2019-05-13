function [tau, tau_fb, System] = Manipulator_OptControl(q, p, tau_c, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    qdot = System.Minv(q) * p;
    %r = System.r(q, qdot);
    Aeq = -System.Psi(q)'*System.Minv(q);
    Beq = -tau_c+System.Psi(q)'*System.Minv(q)*System.Psi(q)*tau_c - System.Psi(q)'*System.Minv(q)*(System.dV(q) + ...
        0.5*System.qdotM(q, qdot)*qdot)+System.Psi(q)'*System.dMinvdt(q, qdot)*p + ...
        System.dPsi(q, qdot)'*qdot + (System.lambda)*System.Psi(q)'*qdot;
    [tau, fval] = quadprog(Aeq'*Aeq, -Beq'*Aeq, [], [],[],[],[],[],[], optimoptions('quadprog','Display','off', 'StepTolerance', 1e-3, 'OptimalityTolerance', 1e-3));
    %[tau, fval] = quadprog(Aeq'*r*r'*Aeq, -(Beq'*r - qdot'*qdot)*r'*Aeq, [], [],[],[],[],[],[], optimoptions('quadprog','Display','off', 'StepTolerance', 1e-5, 'OptimalityTolerance', 1e-5));

    % Add here a condition that guarantuees passivity! S >= 0!
    %tau = inv(Aeq'*Aeq+5e-1)*Aeq'*Beq
    %fprintf('%.2f\n', fval+0.5*(Beq'*r - qdot'*qdot)'*(Beq'*r - qdot'*qdot));
    tau_fb = zeros(2, 1);
end