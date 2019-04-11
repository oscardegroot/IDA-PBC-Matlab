function [V, Vdot] = R_Validation(q, qdot, p, pdot, r, tau, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
     
    V = System.R(q, r);
    
    % dPsi?
    rdot = System.dPsi(q, qdot)'*p + System.Psi(q)'*pdot +...
        System.lambda*System.Psi(q)'*qdot;% + ...
        %System.epsilon*tau;% Change 1
    
    Vdot = rdot'*inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*r;
    Vdot = Vdot + 0.5*r'*System.drLr(q, qdot);
    Vdot = Vdot + tau'*System.epsilon*inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*r;
   
   

end