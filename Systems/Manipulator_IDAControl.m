function [tau, tau_c_out, System] = Manipulator_IDAControl(q, p, tau_c, tau_kv, index, start_delays)

    % tau_kv is hack-and-slashed relative velocity damping

    T = get_param('Model_2Systems_IDA','SimulationTime');
    n_index = index;
    index = num2str(index);
    qdot = inv(feval(['Mm' index], q))*p;

    if(T < start_delays(n_index))
        tau_c = zeros(2, 1);
%         tau_c_out = zeros(2, 1);
%         tau = feval(['dHdq' index], q, qdot);
       % return;
    end

    %% IDA-PBC Control
    Kv = 5*eye(3); % 5
    tau = feval(['dHdq' index], q, qdot) - Kv*qdot + 0.0*tau_kv - 1*feval(['dVs' index], q);
    tau_c_out = tau_c;
    %tau_c_out = getMom(index, q, tau_c);
    
 end
