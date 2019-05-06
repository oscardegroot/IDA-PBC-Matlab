function [z, r, y_d] = UAV_z(q, p, index)

    load(['Systems/UAV_n' num2str(index)]);
    
    z = System.a(q);
    r = System.r(q, System.Minv(q)*p);
    %r = System.lambda*z + ...
    %    inv(System.Psi(q)'*System.Md(q)*inv(System.M(q))*System.Psi(q))*System.Psi(q)'*p;
    y_d = zeros(2,1);%System.Fd(q)'*inv(System.Md(q))*p;
    %r = System.lambda*z + inv(System.Psi(q)'*System.Psi(q) + 0.1*eye(2))*System.Psi(q)'*p;
end