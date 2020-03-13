function [tau, tau_fb, System] = Pendulum_Control(q, p, index)

    %load(['Systems/Pendulum_n' num2str(index)], 'System');
    index = num2str(index);
    %qdot = inv(feval(['Mm' index], q))*p;
    
    tau_fb = 0;
    tau = - feval(['Kv' index], q)*p + feval(['dV' index], q)  - feval(['dVs' index], q) ;

end
