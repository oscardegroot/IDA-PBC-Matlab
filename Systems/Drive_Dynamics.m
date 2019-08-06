function [qdot, pdot] = Drive_Dynamics(q, p, tau_l, tau_c, index)
    
    load(['Systems/Drive_n' num2str(index)]);
    
    qdot = System.Minv(q)*p;
   % pdot = System.F(q)*tau_l + [System.Fd(q)*tau_c; 0];
    pdot = System.F(q)*tau_l + System.F(q)*System.Fd(q)*tau_c;
end


