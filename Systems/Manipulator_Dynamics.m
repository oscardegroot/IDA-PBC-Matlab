function [qdot, pdot] = Manipulator_Dynamics(q, p, tau_l, tau_c, index)
    
    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    qdot = System.Minv(q)*p;
    
    % Assumes F = I
    %det(System.Pcomp(q)) | System.M(q)
     %[~, S, ~] = svd(System.Psi(q));
    %sigma = System.Sigma(q)'*System.Sigma(q)
    %A = inv(eye(3) - sigma*(eye(3) - System.Minv(q)));
    %det(A)
 %A = eye(3);
 %System.M(q)*pinv(System.Psi(q)*System.Psi(q)', 1e-2)
    pdot = (-System.dV(q) - 0.5*System.qdotM(q, qdot)*qdot +...
            tau_l + pinv(System.Psi(q), 1e-5)'*tau_c);%System.Phi(q)
end


