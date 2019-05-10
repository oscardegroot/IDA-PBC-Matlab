function [qdot, pdot] = UAV_Dynamics(q, p, tau_l, tau_c, index)
    
    load(['Systems/UAV_n' num2str(index)]);
    
    qdot = System.Minv(q)*p;
    %inv(System.M(q))*p;
    % Assumes M(q) = M()
    %% Regular
    %pdot = -System.dV(q) + System.F(q)*tau_l + System.Fd(q)*tau_c;
    %% Modified%System.Fd(q)*
    pdot =(-System.dV(q) + System.F(q)*tau_l + pinv(System.Psi(q))'*tau_c);
    %System.F(q)*inv(System.F(q)'*System.F(q))*'
end


