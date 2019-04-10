function [tau, tau_fb] = Pointmass2D_Euler(q, p, index)
    [tau, tau_fb, System] = Pointmass2D_Control(q, p, index);
    
    % Discrete
    tau = tau - System.G*System.Ts*inv(System.M(q))*p;
end