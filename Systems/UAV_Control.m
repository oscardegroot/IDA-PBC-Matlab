function [tau, tau_fb, System] = UAV_Control(q, p, index)

    load(['Systems/UAV_n' num2str(index)]);
    qdot = System.Minv(q)*p;
    
    %% [Main Scheme] Direct Scheme Control zdot
    tau = inv(System.F(q)'*System.F(q))*System.F(q)'*(System.dV(q) ...
        - System.nPsi(q)'*System.dVs(q))...   
        - System.Kv(q)*System.F(q)'*qdot;
    
    tau_fb = -(System.dPsi(q,qdot)'*qdot + (System.lambda + System.gamma)*System.Psi(q)'*qdot)...
           + System.Psi(q)'*System.F(q)*System.Kv(q)*System.F(q)'*qdot;


    %% Accurate local matching
%     tau = inv(System.F(q)'*System.F(q))*System.F(q)'*(System.dV(q) ...
%         - pinv(System.annPsi(q))*(System.anndPsi(q, qdot)*qdot)...
%         - (System.lambda+System.gamma)*qdot - System.dVs(q));
%     
%     tau_fb = -(System.dPsi(q,qdot)'*qdot) + System.Psi(q)'*System.dVs(q);% + (System.lambda + System.gamma)*System.Psi(q)'*qdot);%...
        
    
    %% Altered Tau_fb scheme (Mas problemas)
%     Fn = (eye(3) - System.Fd(q)*System.Psi(q)');
%     tau = inv(System.F(q)'*System.F(q))*System.F(q)'*(...
%         System.dV(q) - Fn*(System.Md(q)*System.dVs(q))...% + System.Md(q)*System.Psi(q)*System.Kv(q)*System.Psi(q)'*qdot));%...
%         - System.annPsi(q)*System.F(q)*System.Kv(q)*System.F(q)'*System.annPsi(q)'*qdot);
%     
%     tau_fb = -System.Kc(q, qdot)*qdot;%...
        %+System.Kv(q)*System.Fd(q)'*qdot;
    
    %det(System.Md(q))
    %System.Fp(q)*(System.dV(q) + System.Fd(q)*tau_fb - System.nPsi(q)*System.dVs(q))
    % Dit is 0 aka de damping wordt volledig verwijdert momenteel
    %- System.F(q)*System.Kv(q)*System.F(q)'*qdot + System.Fd(q)*System.Psi(q)'*System.F(q)*System.Kv(q)*System.F(q)'*qdot
    %System.Fd(q)*System.Psi(q)' % Deze is NIET gelijk aan I
   %- System.F(q)*System.Kv(q)*System.F(q)'*qdot+System.Psi(q)'*(eye(3) - System.Psi(q)*pinv(System.Psi(q)))*System.F(q)*System.Kv(q)*System.F(q)'*qdot % Deze heeft gewoon een waarde
  % Condition number is 1
   %(eye(3) - System.Fd(q)*System.Psi(q)')*System.Psi(q)
    %% LUT Based
    % load('Systems/UAV_n1_LUT');
    % qdot = p; % Because M = I!!!
    % q_i = round((q(3)-LUT.q_low)/LUT.q_res); qdot_i = round((qdot(3)-LUT.qdot_low)/LUT.qdot_res);
    % tau = LUT.A_table(:, :, q_i)*qdot + LUT.B_table(:, :, q_i, qdot_i)*qdot + ...
    %     LUT.D_table(:, q_i);
    % tau_fb = zeros(2,1);

%     %% Strictly Cooperative Control
%     % IDA-PBC
%     tau = inv(System.F(q)'*System.F(q))*System.F(q)'*(System.dV(q) + System.J(q, p)*inv(System.Md(q))*p...
%                        -System.Md(q)*inv(System.M(q))*(0.5*System.dMd_dq(q, p) + System.dVs(q)))...
%           -System.Kv(q)*System.F(q)'*qdot;
%      
%     % Cooperative Manipulation
%     tau_fb = -System.dPsi(q, qdot)'*qdot - (1+System.lambda)*System.Psi(q)'*qdot - ...
%         System.Psi(q)'*(-System.Md(q)*inv(System.M(q))*(0.5*System.dMd_dq(q, p) + System.dVs(q))...
%         + System.J(q,p)*inv(System.Md(q))*p - ...
%         System.F(q)*System.Kv(q)*System.F(q)'*qdot);
    %% Extra control (classic)
    % NEW VALUES (PSI+)
%     tau_fb = System.Psi(q)'*( (0.5*System.dMd_dq(q, p) + System.dVs(q)) -...
%         System.lambda*qdot +inv(System.Md(q))*(System.J(q, p)*inv(System.Md(q))*p-...
%         System.F(q)*System.Kv(q, qdot)*System.F(q)'*inv(System.Md(q))*p)) - ...
%         System.dPsi(q, qdot)'*qdot;
%     tau_fb = System.Psi(q)'*((0.5*System.dMd_dq(q, p) + System.dVs(q)) -...
%         System.lambda*qdot + inv(System.Md(q))*((System.F(q)*System.Kv(q, qdot)*System.F(q)' - ...
%         System.J(q, p))*inv(System.Md(q))*p)) - System.dPsi(q, qdot)'*qdot;
    
    % OLD VALUES
    %tau_fb = inv(System.Psi(q)'*System.Md(q)*inv(System.M(q))*System.Psi(q))*System.Psi(q)'*...
        %System.Md(q)*inv(System.M(q))*(System.dVs(q));   
%     Linv = inv(System.Psi(q)'*System.Md(q)*inv(System.M(q))*System.Psi(q) + System.epsilon*eye(2));
%     tau_fb = -Linv*System.Psi(q)'*...
%         System.J(q, p)*inv(System.Md(q))*p;
%     tau_fb = tau_fb - Linv*System.dPsi(q, qdot)*p;
%     %tau_fb = tau_fb + Linv*System.Psi(q)'*System.Md(q)*inv(System.M(q))*System.dMd_dq(q, p);
%     tau_fb = tau_fb + inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*System.Psi(q)'*System.dMd_dq(q, p);
%     tau_fb = tau_fb + inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*System.Psi(q)'*System.dVs(q);
end