% Not used
function [qddot]  = Manipulator_qddot(q, p, qdot, pdot, index)

    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    
    qddot = System.Minv(q)*(pdot - System.dMdt(q, qdot)*qdot);

end