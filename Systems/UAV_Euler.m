function [tau, tau_fb, tau_ext] = UAV_Euler(q, p, index)
    [tau, tau_fb, System] = UAV_Control(q, p, index);
    
    % Discrete
    tau_ext = -System.G(q)*p;
end