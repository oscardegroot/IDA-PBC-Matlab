function [qdot, pdot] = UAV_Dynamics(q, p, tau_l, tau_c, index)
    
    load(['Systems/UAV_n' num2str(index)]);
    
    qdot = System.M(q)\p;
    %inv(System.M(q))*p;
    % Assumes M(q) = M()
    pdot = -System.dV(q) + System.F(q)*tau_l + System.Fd(q)*tau_c;
end


