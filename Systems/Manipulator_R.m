function [V, Vdot] = Manipulator_R(q, qdot, p, pdot, r, index)

    load(['Systems/manipulator3_n' num2str(index)], 'System');
     
    V = System.R(q, r);
    
    qddot = System.dMinvdt(q,qdot)*p + System.Minv(q)*pdot;
    rdot = System.Psi(q)'*qddot + System.dPsi(q,qdot)'*qdot + ...
        System.lambda*System.Psi(q)'*qdot;
%     rdot = System.Psi(q)'*pdot + System.dPsi(q, qdot)'*p + ...
%         System.lambda*System.Psi(q)'*qdot;
  
    %% For gain (tau_Kv = tau_c)
    % Standard
    Vdot = rdot'*r;
    % Damping (lambda*zdot'z)
    Vdot = Vdot + qdot'*System.Psi(q)*System.lambda*System.a(q);

end