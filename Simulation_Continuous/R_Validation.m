function [V, Vdot, Vcomp] = R_Validation(q, qdot, p, pdot, r, tau_Kv, y_d, Kv, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
     
    aPsi = null(System.Psi(q)');
    V = aPsi'*(pdot + System.lambda*qdot+System.dVs(q));
    
    %System.R(q, r);
    Vcomp = 0;%tau'*System.epsilon*inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*r;
    
    % Dit klopt natuurlijk niet
    %rdot = System.dPsi(q, qdot)'*p + System.Psi(q)'*pdot +...
        %System.lambda*System.Psi(q)'*qdot;
    rdot = System.dPsi(q, qdot)'*p +...
        System.Psi(q)'*pdot + ...
        System.lambda*System.Psi(q)'*qdot;
    % rdot * Linv * r
    Vdot = rdot'*System.Linv(q)*r;
    
    %% For S = 0 check (otherwise leave S terms out)
    Vdot = Vdot + 0.5*r'*System.drLr(q, qdot);
    %Vdot = Vdot + System.lambda*qdot'*System.Psi(q)*System.a(q);
    
    %S = qdot'*System.Psi(q)*System.Psi(q)'*qdot;
    %Vdot = Vdot + S;


end