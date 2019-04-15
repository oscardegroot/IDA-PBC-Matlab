function [tau_c] = Manipulator_Pre(q, tau_k, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    % get rid of the levenberg contribution (by adding another one...)
    tau_c = tau_k + inv(System.Psi(q)'*System.Psi(q) + 1e-3*eye(2))*System.epsilon*tau_k;
    %tau_c = pinv(System.Psi(q)'*System.Psi(q), 1e-2)*(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*tau_k;
end