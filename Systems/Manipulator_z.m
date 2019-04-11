function [z, r, y_d] = Manipulator_z(q, p, tau_i, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    z = System.a(q);
    %r = System.lambda*z + System.Psi(q)'*p;
    r = System.lambda*z + System.Psi(q)'*p;% + System.epsilon*eye(2)*tau_i;
    y_d = System.Psi(q)'*System.Minv(q)*p;
end