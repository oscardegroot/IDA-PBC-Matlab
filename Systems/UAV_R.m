function [V, Vdot] = UAV_R(q, qdot, p, pdot, r, index)

    load(['Systems/uav_n' num2str(index)], 'System');
    
    qddot = pdot;
    
    % Now printing local dynamics
    %theta_ddot = System.anndPsi(q, qdot)*qdot + System.annPsi(q)*qddot;
    V = 0; %theta_ddot + System.annPsi(q)*(10*qdot + System.dVs(q));
    %System.R(q, r);
    
    
    rdot = System.Psi(q)'*qddot + System.dPsi(q,qdot)'*qdot + ...
        System.lambda*System.Psi(q)'*qdot;
  
    % Testje
    %Vdot = pdot'*System.Psi(q)*r;
    
    %% For gain (tau_Kv = tau_c)
    % Standard
    Vdot = rdot'*r;
    % Damping (lambda*zdot'z)
    Vdot = Vdot + System.gamma*qdot'*System.Psi(q)*System.lambda*System.a(q);
    %Vdot = Vdot + 10*qdot'*System.Psi(q)*System.Psi(q)'*qdot;
end