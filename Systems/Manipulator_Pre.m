function [tau_c] = Manipulator_Pre(q, tau_k, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    %tau_c = %tau_k - System.epsilon*inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*tau_k;
    %tau_c = pinv(System.Psi(q)'*System.Psi(q))*(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*tau_k;
    %tau_c = tau_k + pinv(System.Psi(q)'*System.Psi(q))*System.epsilon*tau_k;
    tau_c = tau_k + inv(System.Psi(q)'*System.Psi(q) + 1e-4*eye(2))*System.epsilon*tau_k;
end