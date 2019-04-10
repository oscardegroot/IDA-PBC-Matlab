function [tau, tau_fb, tau_ext] = Manipulator_Euler(q, p, index)
    [tau, tau_fb, System] = Manipulator_Control(q, p, index);
    
    % Discrete
    tau_ext = -System.G*System.Minv(q)*p;
end