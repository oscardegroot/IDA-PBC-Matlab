function [tau, tau_c_out, System] = Manipulator_IDAControl(q, p, tau_c, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    qdot = System.Minv(q) * p;

    %% IDA-PBC Control
    tau = System.dHdq(q, qdot) - System.Kv(q)*qdot;
    tau_c_out = getMom(System, q, tau_c);
    
    end
