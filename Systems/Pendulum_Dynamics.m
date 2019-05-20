function [qdot, pdot] = Pendulum_Dynamics(q, p, tau_l, tau_c, index)
    
    load(['Systems/Pendulum_n' num2str(index)], 'System');
    
    qdot = p;
    
    %% zdot Scheme
    pdot = -System.dV(q) + tau_l;%System.Phi(q)System.M(q)*
%     pdot = (-System.dV(q) - 0.5*System.qdotM(q, qdot)*qdot +...
%                 tau_l + System.M(q)*pinv(System.Psi(q), 1e-5)'*tau_c);
end


