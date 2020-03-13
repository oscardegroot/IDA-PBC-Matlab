function [z_out, r_out] = Manipulator_z(q, p, index)

    %load(['Systems/Manipulator3_n' num2str(index)], 'System');
    index = num2str(index);
    z_out = feval(['a' index], q);
    r_out = feval(['r' index], q, inv(feval(['Mm' index], q))*p);%System.lambda*z + z_dot;
end