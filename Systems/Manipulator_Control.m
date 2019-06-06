function [tau, tau_m, tau_fb, System] = Manipulator_Control(q, p, tau_c, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    qdot = System.Minv(q) * p;

    %% Shaping: zdot + lambda * z
    tau = System.dHdq(q, qdot)...
            - System.M(q)*System.nPsi(q)*(System.dVs(q) + System.Kv(q)*qdot)...
            - System.M(q)*System.dMinvdt(q, qdot)*p...
            - System.M(q)*(System.lambda + 1)*qdot;
    
    %tau_fb = tau_c;
    if(System.isLeader)
        tau_c = tau_c + System.B*(System.lambda*System.target - System.r(q, qdot));
    end
        
    tau_m = getMom(System, q, tau_c);
    tau_fb = tau_m - System.dPsi(q,qdot)'*qdot;
    
    end
