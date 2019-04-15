function [System, SInfo] = UAV_System(lambda, epsilon, index, varargin)

    % This model features a !precompensated! quadrotor.

    g = 9.81;
    e = 0.1;
    k1 = 2;
    k2 = 6.67;%6.67 -> mag helemaal niet toch? (zou tussen 10 en 20 moeten zijn...)
    k3 = 1.02;
    g3 = k1 - e*k2;
    
    System.Ts = RetrieveTs(varargin);
    
    filename = ['Systems/UAV_n' num2str(index)];
    
    % System
    System.M = @(q) eye(3);
    System.n = 3;
    System.name = 'UAV';
    System.dV = @(q) [0;0;-g/e*sin(q(3))];% differs from that in 267 Laurens
    System.F = @(q) [eye(2); [1/e*cos(q(3)) 1/e*sin(q(3))]];
    System.a = @(q) q(1:2) - 1/g3 * [k3*sin(q(3)); (k3-k1*e)*(1-cos(q(3)))];
    System.Psi = @(q) [eye(2); [-k3/g3*cos(q(3)) -(k3-k1*e)/g3*sin(q(3))]];
    System.dPsi = @(q, qdot) [zeros(2, 2) [k3/g3*sin(q(3))*qdot(3); -(k3-k1*e)/g3*cos(q(3))*qdot(3)]];
    % Control
    System.Md = @(q) [k1*e*(cos(q(3)))^2 + k3 k1*e*cos(q(3))*sin(q(3)) k1*cos(q(3));...
                      k1*e*cos(q(3))*sin(q(3)) -k1*e*(cos(q(3)))^2+k3 k1*sin(q(3));...
                      k1*cos(q(3)) k1*sin(q(3)) k2];
    
    %% Find Mdot;
%     q = sym('q', [3, 1]); p = sym('p', [3, 1]);
%     qdot = sym('qdot', [3, 1]);
%     assume([q p qdot], 'real');
%     
%     Md = System.Md([0; 0; q(3)]);
%     Mdinv = simplify(inv(Md));
%     Mdinv = matlabFunction(Mdinv);
%     Mdinv = @(q, qdot) Mdinv(q(1), q(2), q(3), qdot(1), qdot(2), qdot(3));
%     System.Mdinv = Mdinv;
    
    %System.dMd_dq = dMdq(Md, q(3));
        %System.Fp = @(q) [cos(q(3)) sin(q(3)) -e];
    % Find Mdot directly, calculate and simplify Md inverse then 
    % validate on single-agent
    
    Find_dMdq;
    System.dMd_dq = dMd_dq;   

    
    
    J1 = @(q, p) p'*inv(System.Md(q)) * [-2*e*cos(q(3)); 2*e*sin(q(3)); 1];
    J2 = @(q, p) p'*inv(System.Md(q)) * [0; 1; 0];
    J3 = @(q, p) p'*inv(System.Md(q)) * [-1; 0; 0];
    System.J = @(q, p) -k1*g3/2*[0 J1(q, p) J2(q, p); -J1(q, p) 0 J3(q, p); -J2(q, p) -J3(q, p) 0];
    System.dVs = @(q) [0; 0; g/g3*sin(q(3))];
    %System.Kv = @(q, p)eye(2);
    System.Phi = @(q) (System.F(q)'*System.F(q))*System.F(q)'*System.Md(q)*inv(System.M(q))*System.Psi(q);
    
    %% R-passivity components
    System.lambda = lambda;
    System.epsilon = epsilon;
    
    System.Fd = @(q) System.Md(q)*inv(System.M(q))*System.Psi(q);
%     System.Kv = @(q, p) eye(2);%System.Phi(q)*(lambda*System.Psi(q)' - 0.5*System.dMd_dq(p, q)'*inv(System.M(q))*p)...
%      *inv(Fd(q)*Fd(q)' + epsilon*eye(3))*Fd(q)*System.Phi(q)';
    
     % The right damping (supposively)
%      System.Kv = @(q, p) System.Phi(q)*inv(System.Fd(q)'*System.Fd(q) + System.epsilon * eye(2))*...
%          (lambda*eye(2) + inv(System.Fd(q)'*System.Fd(q) + System.epsilon * eye(2))...
%      * System.Fd(q)'*System.J(q, p)*inv(System.Fd(q)*System.Fd(q)' + ...
%          System.epsilon*eye(3))*System.Fd(q))*System.Phi(q)';
    Finv = @(q) inv(System.Fd(q)'*System.F(q) + epsilon*eye(2));

    %System.Kv = @(q, p) eye(2);%System.F(q)'*Finv(q) * lambda*eye(3) * Finv(q)*System.F(q);
    System.Kv = @(q, p) lambda*Finv(q)*System.Fd(q)'*inv(System.M(q))*System.Md(q)*System.Fd(q)*Finv(q)';
    System.R = @(q, r) 0.5*r'*inv(System.Psi(q)'*System.Md(q)*inv(System.M(q))*System.Psi(q) + System.epsilon*eye(2))*r;

    % Discrete time compensation
    if(System.Ts > 0)
        Lv = @(q) eye(3);
        kappa = 0;
        System.G = @(q) System.Ts*kappa*inv(System.F(q)'*System.F(q))*System.F(q)'*...
            System.Md(q)*inv(System.M(q))*Lv(q)*inv(System.M(q));
    end
    % Mdinv * Ig * Md*Minv*Lv*Minv > 0
    
    %SolveForLv;
    %% Define info for this system
    SInfo.n = 3;
    SInfo.name = 'UAV';
    SInfo.legend = {[SInfo.name ' x'], [SInfo.name ' y'], [SInfo.name ' \theta']};
    SInfo.plotf = @PlotUAV;
    SInfo.filename = filename;
    SInfo.h = e;
    SInfo.l = e;
   
    save(filename, 'System');
end