function [z, r, z_dot] = Manipulator_z(q, p, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    z_dot = System.Psi(q)'*System.Minv(q)*p;
    z = System.a(q);
    r = System.r(q, System.Minv(q)*p);%System.lambda*z + z_dot;
end