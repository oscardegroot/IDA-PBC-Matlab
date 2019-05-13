function [V, Vdot] = UAV_R(q, qdot, p, pdot, r, index)

    load(['Systems/uav_n' num2str(index)], 'System');
    
    V = System.R(q, r);
    
    qddot = pdot;
    
    rdot = System.Psi(q)'*qddot + System.dPsi(q,qdot)'*qdot + ...
        System.lambda*System.Psi(q)'*qdot;
  
    % Testje
    %Vdot = pdot'*System.Psi(q)*r;
    
    %% For gain (tau_Kv = tau_c)
    % Standard
    Vdot = rdot'*r;
    % Damping (lambda*zdot'z)
    Vdot = Vdot + qdot'*System.Psi(q)*System.lambda*System.a(q);
    %Vdot = Vdot + 10*qdot'*System.Psi(q)*System.Psi(q)'*qdot;
end