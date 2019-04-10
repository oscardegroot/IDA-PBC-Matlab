function [dA] = dAdq(A, n)
    q = sym('q', [n, 1]);
    dA = [];
    for i = 1 : n
        dA = [dA diff(A, q(i))];
    end

end