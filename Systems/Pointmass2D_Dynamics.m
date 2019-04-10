function [qdot, pdot] = Pointmass2D_Dynamics(q, p, tau_l, tau_c, index)
    
    load(['Systems/2D_n' num2str(index)], 'System');
    
    qdot = inv(System.M(q))*p;
    pdot = -System.dV(q) + System.F(q)*(tau_l + System.Phi(q)*tau_c);
end