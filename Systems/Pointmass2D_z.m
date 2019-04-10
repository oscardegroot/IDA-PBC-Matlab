function [z, r, y_d] = Pointmass2D_z(q, p, index)

    load(['Systems/2D_n' num2str(index)], 'System');
    
    z = System.a(q);
    r = System.lambda*z + System.Psi(q)'*p;
    y_d = System.Psi(q)'*inv(System.M(q))*p;
end