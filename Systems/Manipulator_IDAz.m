function [z, zdot] = Manipulator_IDAz(q, p, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    z = System.a(q);
    zdot = System.Psi(q)'*System.Minv(q)*p;
end