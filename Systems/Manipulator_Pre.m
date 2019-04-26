function [tau_c] = Manipulator_Pre(q, tau_k, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    u_max = 20;
    %% Transpose Strategy
    %tau_c = tau_k;%*(System.Sigma(q)'*System.Sigma(q));
    
    %% Baseline levenberg
    %tau_c = tau_k + inv(System.Psi(q)'*System.Psi(q) + 1e-3*eye(2))*System.epsilon*tau_k;
    
    %% Iterative levenberg (input explodes)
    epsilon2 = 1e-1;
    N = 1000;
    Linv = inv(System.Psi(q)'*System.Psi(q) + epsilon2*eye(2));
   
    tau_c = Linv*System.epsilon*tau_k;
    tau_diff = 1e99;
    % tau is 2 dimensional!
    for i = 1 : N
        tau_c_prev = tau_c;
        tau_c = Linv*(System.epsilon*tau_k + epsilon2*tau_c);
        
        % If the iteration diverges, stop and return the initial value
        if(tau_diff < abs(tau_c - tau_c_prev))
            tau_c = Linv*System.epsilon*tau_k;
            fprintf('[Diverged]\n');
            break;
        end
        
%         if(abs(tau_c) > 50)
%             tau_c = 0;
%             fprintf('Too large!\n');
%             break;
%         end
        % If it converged, stop iterating
        tau_diff = abs(tau_c - tau_c_prev);
        if(tau_diff < 1e-4)
            fprintf('[Converged] Converged after %.0f steps!\n', i);
            break;
        end
        
         
         if(i == N)
             fprintf('[Max Iterations] Difference: %.4f\n', tau_diff);
%              if(tau_diff > 1e-3)
%                 tau_c = 0;
%              end
         end
    end
    tau_c = tau_c + tau_k;
    
    for i = 1 : 2
        tau_c(i) = min(max(-u_max, tau_c(i)), u_max);
    end

end