function [tau_c] = Manipulator_Pre(q, tau_k, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    % get rid of the levenberg contribution (by adding another one...)
    %tau_c = tau_k + inv(System.Psi(q)'*System.Psi(q) + 1e-3*eye(2))*System.epsilon*tau_k;
    Linv = inv(System.Psi(q)'*System.Psi(q) + 1e-3*eye(2));
    tau_c = Linv*System.epsilon*tau_k;
    tau_c_prev = 1e99;
    for i = 1 : 5
        tau_c_prev = tau_c;
        tau_c = Linv*(System.epsilon*tau_k + 1e-3*tau_c);
        
        if(abs(tau_c - tau_c_prev) > 1e-3)
            break;
        end
    end
    tau_c = tau_c + tau_k;

end