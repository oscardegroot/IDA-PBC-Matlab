function [V, Vdot, S] = R_Validation(q, qdot, p, pdot, r, tau_Kv, y_d, Kv, index)

    load(['Systems/uav_n' num2str(index)], 'System');
     
    %aPsi = null(System.Psi(q)');
    V = 0;%0.5*p'*System.Minv(q)*p;%aPsi'*(pdot + System.lambda*qdot+System.dVs(q));
    
    %System.R(q, r);
    S = 0;%qdot'*System.Kv(q)*qdot;%tau'*System.epsilon*inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*r;
    
    qddot = pdot;%System.dMinvdt(q,qdot)*p + System.Minv(q)*pdot;
    
    rdot = System.Psi(q)'*qddot + System.dPsi(q,qdot)'*qdot + ...
        System.lambda*System.Psi(q)'*qdot;
    
%              rdot = System.dPsi(q, qdot)'*p +...
%          System.Psi(q)'*pdot + ...
%          System.lambda*System.Psi(q)'*qdot;
    % rdot * Linv * r
    %% For gain (tau_Kv = tau_c)
    % Standard
    Vdot = rdot'*r;
    % Damping (lambda*zdot'z)
    %Vdot = Vdot + qdot'*System.Psi(q)*System.lambda*System.a(q);
    
    %% For S = 0 check (otherwise leave S terms out)
    %Vdot = rdot'*System.Linv(q)*r;
    %Vdot = Vdot + 0.5*r'*System.drLr(q, qdot);
    %Vdot = Vdot + System.lambda*qdot'*System.Psi(q)*System.a(q);
    
    %S = qdot'*System.Psi(q)*System.Psi(q)'*qdot;
    %Vdot = Vdot + S;


end