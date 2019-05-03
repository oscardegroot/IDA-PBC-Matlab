function [tau, tau_fb, System] = Manipulator_Control(q, p, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    qdot = System.Minv(q) * p;
    tau = System.dV(q) - System.dVs(q); %- System.dVs(q) - System.Kv(q, qdot)*qdot;
    
    %% Method 1) Pseudo-Inverse
    %pinv(System.Psi(q))
    %tau_fb = lsqminnorm(System.Psi(q),System.Kv(q,qdot)*qdot + System.dVs(q)+0.5*System.qdotM(q, qdot)*qdot) - ...
    %    (System.dPsi(q, qdot)'*qdot + System.lambda*System.Psi(q)'*qdot) + System.Psi(q)'*qdot;
    
    %% Direct Method with psi'p
%     tau = tau - (System.lambda)*qdot;
%     tau_fb = -System.dPsi(q,qdot)'*p + System.Psi(q)'*System.dVs(q);
    
    %% Direct Method with zdot
    %tau = tau - System.dMinvdt(q, qdot)*p - (System.lambda + 1)*qdot;
    tau = tau + 0.5*System.qdotM(q, qdot)*qdot...
            - System.dMinvdt(q, qdot)*p...
            - (System.lambda + 1)*qdot;
    tau_fb = -System.dPsi(q,qdot)'*qdot + System.Psi(q)'*System.dVs(q);
    
    % Linv Psi'(M-I)qddot
    %tau_fb = System.Linv(q)*System.Psi(q)'*(System.M(q) - eye(3))*...
    %    System.dMinvdt(q, qdot)*p;
    
    % Psi_dot * p term
    %tau_fb = -System.Linv(q)*System.dPsi(q, qdot)'*p;
    
    % Psi+ * dVs term
    %tau_fb = tau_fb + System.Linv(q)*System.Psi(q)'*System.dVs(q);
    
    % dV/dq term
    %tau_fb = tau_fb - 0.5*System.drLr(q, qdot);
    
    % damping feedback
    %tau_fb = tau_fb - System.Psi(q)'*qdot;
end