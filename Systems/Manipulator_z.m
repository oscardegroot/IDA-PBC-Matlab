function [z, r, y_d] = Manipulator_z(q, p, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    z = System.a(q);
    %r = System.lambda*z + System.Psi(q)'*p;
    r = System.lambda*z + System.Psi(q)'*p;
    y_d = System.Psi(q)'*System.Minv(q)*p;
end