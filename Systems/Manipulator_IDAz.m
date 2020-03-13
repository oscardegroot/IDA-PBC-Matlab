function [z, zdot] = Manipulator_IDAz(q, p, index)
    
    index = num2str(index);
    %load(['Systems/Manipulator3_n' num2str(index)], 'System');
    z = feval(['a' index], q);%System.a(q);
    zdot = inv(feval(['Mm' index], q))*p;
end