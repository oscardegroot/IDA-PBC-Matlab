function [tau, tau_fb, System] = Manipulator_Control(q, p, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    qdot = System.Minv(q) * p;
    %
    tau = System.dV(q) + 0.5*System.qdotM(q, qdot)*qdot...
            - System.M(q)*System.nPsi(q)*(System.dVs(q) - p)...
            - System.M(q)*System.dMinvdt(q, qdot)*p...
            - System.M(q)*(System.lambda + 1)*qdot;%...
            %- p;
    tau_fb = -System.dPsi(q,qdot)'*qdot;% + System.Psi(q)'*qdot;
%     tau = System.dV(q)+0.5*System.qdotM(q, qdot)*qdot...
%             - System.M(q)*System.dMinvdt(q, qdot)*p...
%             - System.M(q)*null(System.Psi(q)')*null(System.Psi(q)')'*System.dVs(q)...
%             - System.M(q)*(System.lambda + 1)*qdot;
%     tau_fb = -System.dPsi(q,qdot)'*qdot;
    

%a = null(System.Psi(q)')*null(System.Psi(q)')';
    %b = (eye(3) - pinv(System.Psi(q))'*System.Psi(q)');
    %a-b
    %% Direct Method with psi'p
%     tau = tau - (System.lambda)*qdot;
%     tau_fb = -System.dPsi(q,qdot)'*p + System.Psi(q)'*System.dVs(q);
    
    %% Direct Method with zdot
   % tau = tau - System.dMinvdt(q, qdot)*p - (System.lambda + 1)*qdot;
%    System.dMinvdt(q, qdot)*p
%     tau = System.dV(q)...
%         - null(System.Psi(q)')*null(System.Psi(q)')'*System.dVs(q)...
%             - System.dMinvdt(q, qdot)*p...
%             - (System.lambda + 1)*qdot
%     tau_fb = -System.dPsi(q,qdot)'*qdot;

%% Shitty new
%     tau = System.dV(q)+0.5*System.qdotM(q, qdot)*qdot...
%             - System.M(q)*System.dMinvdt(q, qdot)*p...
%             - System.M(q)*null(System.Psi(q)')*null(System.Psi(q)')'*System.dVs(q)...
%             - System.M(q)*(System.lambda + 1)*qdot;
%     tau_fb = -System.dPsi(q,qdot)'*qdot;

    %% Psi'*p
%     tau = System.dV(q)+ 0.5*System.qdotM(q, qdot)*qdot...
%         - (System.lambda + 1)*qdot...
%         - null(System.Psi(q)')*null(System.Psi(q)')'*...
%         (System.dVs(q) + 0.5*System.qdotM(q, qdot)*qdot);
%     tau_fb = System.dPsi(q,qdot)'*p;%+ 0.5*System.qdotM(q, qdot)*qdot
    %...
    
    
    
    % Psi_dot * p term
    %tau_fb = -System.Linv(q)*System.dPsi(q, qdot)'*p;
    
    % Psi+ * dVs term
    %tau_fb = tau_fb + System.Linv(q)*System.Psi(q)'*System.dVs(q);
    
    % dV/dq term
    %tau_fb = tau_fb - 0.5*System.drLr(q, qdot);
    
    % damping feedback
    %tau_fb = tau_fb - System.Psi(q)'*qdot;
end