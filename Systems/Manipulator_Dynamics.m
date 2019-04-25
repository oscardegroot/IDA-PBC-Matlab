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
    pdot = System.M(q)*(-System.dV(q) - 0.5*System.dMdq(q, qdot) +...
            tau_l + System.Phi(q)*tau_c);
end


