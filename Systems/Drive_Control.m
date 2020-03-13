function [tau, tau_fb, System] = Drive_Control(q, p, index)

    load(['Systems/Drive_n' num2str(index)]);
    %qdot = p;
    
    %% [Main Scheme] Direct Scheme Control zdot
    tau = -System.Kv(q)*p;
    
    tau_fb = -(System.lambda + System.gamma)*System.Psi(q)'*p;%...
            %+ System.Psi(q)'*System.Kv(q)*p;
         
        
        
        %tau = -System.Kv(q)*inv(System.S(q)'*System.M(q)*System.S(q))*System.S(q)'*p;
    %tau_fb = -0.1*(System.lambda + System.gamma)*System.Psi(q)'*qdot;%...
        %+ System.Psi(q)'*System.F(q)*System.Kv(q)*System.F(q)'*qdot;

end