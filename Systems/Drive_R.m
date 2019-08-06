function [V, Vdot] = Drive_R(q, qdot, p, pdot, r, index)

    load(['Systems/Drive_n' num2str(index)], 'System');
    
    qddot = pdot;
    V = 0;
    
    rdot = System.Psi(q)'*qddot + ...
        System.lambda*System.Psi(q)'*qdot;
  %System.dPsi(q,qdot)'*qdot + ...
   
    %% For gain (tau_Kv = tau_c)
    % Standard
    Vdot = rdot'*r;
    % Damping (lambda*zdot'z)
    Vdot = Vdot + System.gamma*qdot'*System.Psi(q)*System.lambda*System.a(q);
end