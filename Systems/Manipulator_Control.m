function [tau, tau_fb, tau_Kv, System] = Manipulator_Control(q, p, zdot, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    qdot = System.Minv(q) * p;
    tau = System.dV(q) - System.dVs(q) - System.Kv(q, qdot)*qdot;

    % Psi_dot * p term
    tau_fb = -inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*...
     System.dPsi(q, qdot)'*System.M(q)*qdot;
    
    % Psi+ * dVs term
    tau_fb = tau_fb + inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*System.Psi(q)'*...
        (System.dVs(q));
    
    % dV/dq term
    tau_fb = tau_fb - 0.5*System.drLr(q, qdot);
    
    % damping feedback
    %tau_fb = tau_fb - System.Linv(q, qdot)*System.Psi(q)'*qdot;
    
    tau_Kv = zeros(2,1);%0.5*System.drLr(q, qdot);
end