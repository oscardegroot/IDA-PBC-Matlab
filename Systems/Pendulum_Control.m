function [tau, tau_fb, System] = Pendulum_Control(q, p, index)

    load(['Systems/Pendulum_n' num2str(index)], 'System');
    
    tau_fb = 0;
    tau = System.dV(q) - System.dVs(q) - System.Kv*p;

end
