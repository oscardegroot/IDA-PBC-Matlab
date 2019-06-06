function [qdot, pdot] = UAV_Dynamics(q, p, tau_l, tau_c, index)
    
    load(['Systems/UAV_n' num2str(index)]);
    
    qdot = System.Minv(q)*p;
    %pdot =(-System.dV(q) + System.F(q)*tau_l + pinv(System.Psi(q))'*tau_c);
    pdot = -System.dV(q) + System.F(q)*tau_l + System.Fd(q)*tau_c;
end


