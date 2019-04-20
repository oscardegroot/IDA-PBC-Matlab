function [z, r, z_dot] = Manipulator_z(q, p, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    z = System.a(q);
    
    z_dot = System.Psi(q)'*System.Minv(q)*p;
    r = System.lambda*z + z_dot;
end