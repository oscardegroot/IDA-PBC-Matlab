function [tau, tau_m, tau_fb, System] = Manipulator_Control(q, p, tau_c, index, start_delays)

   % load(['Systems/Manipulator3_n' num2str(index)], 'System');
    T = get_param('Model_2Systems','SimulationTime');
    n_index = index;
    index = num2str(index);
    if(T < start_delays(n_index))
        tau_c = zeros(2, 1);
%         tau_fb = zeros(2, 1);
%         qdot = inv(feval(['Mm' index], q))*p;
%         tau = feval(['dHdq' index], q, qdot); % Compensate for movement
%         tau_m = zeros(2, 1);
        %return;
    end
    
    qdot = inv(feval(['Mm' index], q))*p;
    % DT gamma: 3, kappa: 1
    Kv = feval(['ddVs' index], q) + eye(3)- inv(feval(['Mm' index], q))*feval(['Mdot' index], q, qdot);%feval(['nPsi' index], q)*(1*eye(3) - inv(feval(['Mm' index], q))*feval(['Mdot' index], q, qdot)) + feval(['dnPsi' index], q, qdot);
    Kz = feval(['Psi' index], q)'*((3 + 1) * eye(3) - inv(feval(['Mm' index], q))*feval(['Mdot' index], q, qdot)) + feval(['dPsi' index], q, qdot)';
    %% Shaping: zdot + lambda * z
    tau_l = Kv*qdot + feval(['dVs' index], q) + ...
        feval(['dnPsi' index], q, qdot) * (feval(['dVs' index], q) + qdot);
    
    tau = feval(['dHdq' index], q, qdot);
    tau = tau - feval(['Mm' index], q)*feval(['nPsi' index], q)*tau_l;%pinv(feval(['nPsi' index], q))*tau_l;
    
    % Are rotations passive??
    %tau_m = tau_c;
    eta = 1;
    tau_m = getMom(index, q, (1/eta)*tau_c);

    tau_fb = tau_m - Kz*qdot;
end
    

 %- System.M(q)*System.Kv(q, qdot)*qdot...           
    %- System.M(q)*System.nfeval(['Psi' index], q)*System.dVs(q);%...

        
            %- System.M(q)*System.nfeval(['Psi' index], q)*(System.dVs(q) + System.Kv(q, qdot)*qdot);%...
%             - System.M(q)*System.dMinvdt(q, qdot)*p...
%             - System.M(q)*(System.lambda + 1)*qdot;
    
    %eig(null(System.feval(['Psi' index], q)')*null(System.feval(['Psi' index], q)')'*(System.dMinvdt(q, qdot) * System.M(q) ...
    %   - System.Kv(q, qdot)))
    %tau_fb = tau_c;
%     if(System.isLeader)
%         tau_c = tau_c + System.B*(System.lambda*System.target - System.r(q, qdot));
%     end
        
