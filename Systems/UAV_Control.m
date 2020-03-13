function [tau, tau_fb, matching, System] = UAV_Control(q, p, index)

    load(['Systems/UAV_n' num2str(index)]);
    qdot = System.Minv(q)*p;
    matching = 0;
    %% [Main Scheme] Direct Scheme Control zdot
%     tau = inv(System.F(q)'*System.F(q))*System.F(q)'*(System.dV(q) ...
%         - System.nPsi(q)'*System.dVs(q))...   
%         - System.Kv(q)*System.F(q)'*qdot;
%     
%     tau_fb = -(System.dPsi(q,qdot)'*qdot + (System.lambda + System.gamma)*System.Psi(q)'*qdot)...
%            + System.Psi(q)'*System.F(q)*System.Kv(q)*System.F(q)'*qdot;


    %% Full cancellation IDA-PBC
    tau = inv(System.F(q)'*System.F(q))*System.F(q)'*(System.dV(q)...
        + (eye(3) - pinv(System.Psi(q)')*System.Psi(q)')*(System.J(q, p)*inv(System.Md(q))*p + ...
        System.Md(q)*System.dHdq(q)));

    %+ System.gamma
    tau_fb = -(System.dPsi(q,qdot)'*qdot + (System.lambda + System.gamma)*System.Psi(q)'*qdot);%...
%            + System.Psi(q)'*System.F(q)*System.Kv(q)*System.F(q)'*qdot;

    %% Accurate local matching
%     tau = inv(System.F(q)'*System.F(q))*System.F(q)'*(System.dV(q) ...
%         - System.nPsi(q)'*System.dVs(q) -...
%         System.F(q)*System.Kv(q)*System.F(q)'*p);
%     
%     tau_fb = -System.dPsi(q,qdot)'*qdot - (System.lambda + System.gamma)*System.Psi(q)'*qdot...
%         + System.Psi(q)'*inv(System.M(q)) * System.F(q)*System.Kv(q)*System.F(q)'*p;
% 
%     matching = System.Fp(q)*(System.dV(q) ...
%         - System.nPsi(q)'*System.dVs(q) -...
%         System.F(q)*System.Kv(q)*System.F(q)'*p + System.Fd(q)*tau_fb);
    
end