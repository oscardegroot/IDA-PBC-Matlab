%% Calculate ddt of a system with 3 coordinates and return it as matlabfunction
function [dF] = ddt(F)
    syms q1(t) q2(t) q3(t) qdot1 qdot2 qdot3
    q1 = q1(t); q2 = q2(t); q3 = q3(t);
    F = subs(F);
    dF = diff(F, t);
    dF = subs(dF,{diff(q1, t), diff(q2, t), diff(q3, t)},{'qdot1', 'qdot2', 'qdot3'}); 
    dF = subs(dF,{q1, q2, q3},{'q1', 'q2', 'q3'});
    
    %syms q1 q2 q3 qdot1 qdot2 qdot3
    %dF = matlabFunction(dF, 'Vars', {[q1; q2; q3],[qdot1; qdot2; qdot3]});
    %dF = @(q, qdot) dF(q, qdot);%dF(q(1), q(2), q(3), qdot(1), qdot(2), qdot(3));
end