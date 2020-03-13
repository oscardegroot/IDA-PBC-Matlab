function [qdot, pdot] = Manipulator_Dynamics(q, p, tau_l, tau_c, index)
    
    %load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    %% zdot Scheme
    index = num2str(index);
    qdot = inv(feval(['Mm' index], q))*p;
    pdot = -feval(['dHdq' index], q, qdot) + tau_l + feval(['Mm' index], q)*...
        pinv(feval(['Psi' index], q)')*tau_c;
end


