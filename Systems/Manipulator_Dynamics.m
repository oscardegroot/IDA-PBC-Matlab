function [qdot, pdot] = Manipulator_Dynamics(q, p, tau_l, tau_c, index)
    
    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    qdot = System.Minv(q)*p;
    
    %% Psi'p Scheme
%     pdot = (-System.dV(q) - 0.5*System.qdotM(q, qdot)*qdot +...
%             tau_l + pinv(System.Psi(q), 1e-5)'*tau_c);%System.Phi(q)
        
    %% zdot Scheme
    pdot = System.M(q)*(-System.dV(q) - 0.5*System.qdotM(q, qdot)*qdot +...
            tau_l + pinv(System.Psi(q), 1e-5)'*tau_c);%System.Phi(q)
end


