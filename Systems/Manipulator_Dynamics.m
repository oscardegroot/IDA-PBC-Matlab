function [qdot, pdot] = Manipulator_Dynamics(q, p, tau_l, tau_c, index)
    
    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    %% zdot Scheme
    qdot = System.Minv(q)*p;
    pdot = (-System.dHdq(q, qdot) + tau_l + System.M(q)*pinv(System.Psi(q)')*tau_c);
end


