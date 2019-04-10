function [tau, tau_fb, System] = Pointmass2D_Control(q, p, index)
    
    load(['Systems/2D_n' num2str(index)], 'System');
    
    tau = System.dV(q)-System.dVs(q)-System.Kv*inv(System.M(q))*p;
    tau_fb = zeros(2,1);

end