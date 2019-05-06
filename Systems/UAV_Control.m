function [tau, tau_fb, System] = UAV_Control(q, p, index)
    
    load(['Systems/UAV_n' num2str(index)]);
    qdot = System.Minv(q)*p;
    
        %% Direct Scheme Control
%     tau = inv(System.F(q)'*System.F(q))*System.F(q)'*(...
%         System.dV(q) - System.dVs(q) - System.lambda*qdot);
%     tau_fb = (-System.dPsi(q,qdot)'*qdot);

    %% Local control (classic)
    tau = inv(System.F(q)'*System.F(q))*System.F(q)'*(System.dV(q) + System.J(q, p)*inv(System.Md(q))*p...
                       -System.Md(q)*inv(System.M(q))*(0.5*System.dMd_dq(q, p) + System.dVs(q)))...
          -System.Kv(q, qdot)*System.F(q)'*inv(System.Md(q))*p;
      

    %% Extra control (classic)
    % NEW VALUES (PSI+)
%     tau_fb = System.Psi(q)'*( (0.5*System.dMd_dq(q, p) + System.dVs(q)) -...
%         System.lambda*qdot +inv(System.Md(q))*(System.J(q, p)*inv(System.Md(q))*p-...
%         System.F(q)*System.Kv(q, qdot)*System.F(q)'*inv(System.Md(q))*p)) - ...
%         System.dPsi(q, qdot)'*qdot;
    tau_fb = System.Psi(q)'*((0.5*System.dMd_dq(q, p) + System.dVs(q)) -...
        System.lambda*qdot + inv(System.Md(q))*((System.F(q)*System.Kv(q, qdot)*System.F(q)' - ...
        System.J(q, p))*inv(System.Md(q))*p)) - System.dPsi(q, qdot)'*qdot;
    
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