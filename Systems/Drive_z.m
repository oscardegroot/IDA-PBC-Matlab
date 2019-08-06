function [z, r] = Drive_z(q, p, index)

    load(['Systems/Drive_n' num2str(index)]);
    
    z = System.a(q);
    r = z;%System.r(q, System.Minv(q)*p);
end