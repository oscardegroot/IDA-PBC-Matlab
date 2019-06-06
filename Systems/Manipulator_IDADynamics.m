function [qdot, pdot] = Manipulator_IDADynamics(q, p, tau_l, tau_c, index)
    
    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    %% zdot Scheme
    qdot = System.Minv(q)*p;
    pdot = (-System.dHdq(q, qdot) + tau_l + System.Psi(q)*tau_c);
end


