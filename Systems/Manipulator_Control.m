function [tau, tau_fb, System] = Manipulator_Control(q, p, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    qdot = System.Minv(q) * p;
    tau = System.dV(q) - System.dVs(q) - System.Kv(q, qdot)*qdot;

    qddot = -System.Minv(q)*System.dMdt(q, qdot)*System.Minv(q)*p;
    
    % Linv Psi'(M-I)qddot
    tau_fb = System.Linv(q)*System.Psi(q)'*(System.M(q) - eye(3))*qddot;
    
    % Psi_dot * p term
    tau_fb =  tau_fb -System.Linv(q)*System.dPsi(q, qdot)'*qdot;
    
    % Psi+ * dVs term
    tau_fb = tau_fb + System.Linv(q)*System.Psi(q)'*System.dVs(q);
    
    % dV/dq term
    tau_fb = tau_fb - 0.5*System.drLr(q, qdot);
    
    % damping feedback
    tau_fb = tau_fb - System.Psi(q)'*qdot;
end