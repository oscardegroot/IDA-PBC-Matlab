function [V, Vdot] = R_Validation(q, qdot, p, pdot, r, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
     
    V = System.R(q, r);
    
    rdot = System.dPsi(q, qdot)'*p + System.Psi(q)'*pdot +...
        System.lambda*System.Psi(q)'*qdot;
    
    Vdot = rdot'*inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*r;
    Vdot = Vdot + r'*System.drLr(q, qdot);
   
   

end