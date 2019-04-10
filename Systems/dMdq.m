% Calculate d(q'*M(q)*q)/dq for M a matlabfunction
function [Y] = dMdq(M, q_n)
    %q = sym('q', [3, 1]); p = sym('p', [3, 1]);
    qdot = sym('qdot', [3, 1]);
    assume(qdot, 'real');
    
    Y = simplify(0.5*[0; 0; qdot'*diff(M, q_n)*qdot]);
    Y = matlabFunction(Y);
    Y = @(q, qdot) Y(q(3), qdot(1), qdot(2), qdot(3));
    %dMdq = simplify(diff(qMq, q_n));
    %Y = subs(dMdq,{diff(q_n, t), diff(q2, t), diff(q3, t)},{'qdot1', 'qdot2', 'qdot3'}); 
    
end