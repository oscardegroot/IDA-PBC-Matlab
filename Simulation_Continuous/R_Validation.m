function [V, Vdot, Vcomp] = R_Validation(q, qdot, p, pdot, r, tau_Kv, y_d, Kv, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
     
    V = System.R(q, r);
    Vcomp = 0;%tau'*System.epsilon*inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*r;
    
    rdot = System.dPsi(q, qdot)'*p + System.Psi(q)'*pdot +...
        System.lambda*System.Psi(q)'*qdot;
    
    % rdot * Linv * r
    Vdot = rdot'*inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*r;
    
    % dV/dq
    Vdot = Vdot + 0.5*r'*System.drLr(q, qdot);
    Vdot = Vdot + tau_Kv'*y_d - y_d'*Kv*y_d - qdot'*System.Kv(q, qdot)*qdot;
    %Vdot = Vdot + tau'*System.epsilon*inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*r;
   
   

end