function [qdot, pdot] = Manipulator_Dynamics(q, p, tau_l, tau_c, index)
    
    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    qdot = System.Minv(q)*p;
    
    % Assumes F = I
    %det(System.Pcomp(q))
    pdot = System.M(q)*(-System.dV(q) - 0.5*System.dMdq(q, qdot) +...
            tau_l + System.Phi(q)*tau_c);
end


