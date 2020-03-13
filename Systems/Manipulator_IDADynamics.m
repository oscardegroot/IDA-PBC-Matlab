function [qdot, pdot] = Manipulator_IDADynamics(q, p, tau_l, tau_c, index)
        
    %% zdot Scheme
    index = num2str(index);
    qdot = inv(feval(['Mm' index], q))*p;
    
    % Pseudo Inverse Jacobian
    %pdot = -feval(['dHdq' index], q, qdot) + tau_l + pinv(feval(['Psi' index], q)') * tau_c;%feval(['Psi' index], q)*tau_c;
    
    % Transpose Jacobian
    pdot = -feval(['dHdq' index], q, qdot) + tau_l + feval(['Psi' index], q) * tau_c;

end


