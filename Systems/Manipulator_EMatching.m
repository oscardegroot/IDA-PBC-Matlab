function [tau, tau_fb, tau_ext] = Manipulator_EMatching(q, p, qddot, index)
    [tau, tau_fb, System] = Manipulator_Control(q, p, index);
    
    % Discrete
    qdot = System.Minv(q)*p;
    tau_ext = System.Ts/2*(System.dVdt(q, qdot) - System.dVsdt(q, qdot)) - ...
        0.1*System.lambda*eye(3)*qddot;%-System.G*System.Minv(q)*p;
    % This could also calculate the acceleration (I'm not measuring
    % anything but velocities)
end