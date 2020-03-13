function [System, SInfo] = Drive_System(lambda, index, varargin)

    % This model features a !precompensated! quadrotor.
    m = 0.1;
    I = 1e-4;
    
    System.Ts = RetrieveTs(varargin);
    
    filename = ['Systems/Drive_n' num2str(index)];
    
    % System
    System.M = @(q) eye(2);
    %System.S = @(q) [cos(q(3)) 0; sin(q(3)) 0; 0 1];
    %System.MSinv = @(q) inv(System.S(q)'*System.M(q)*System.S(q));
    System.Minv = @(q) inv(System.M(q));
    System.n = 2;
    System.name = 'Differential Drive';
    System.dV = @(q) [0;0];
    System.F = @(q) eye(2);%@(q) [cos(q(3)) 0; sin(q(3)) 0; 0 1];
    %System.A = @(q) [sin(q(3)); -cos(q(3)); 0];
    
    %System.Fp = @(q) [sin(q(3)) -cos(q(3)) 0];
    System.dHdq = @(q) 0;
    %% No null space, but Md
    %e = 0.05;
    %System.a = @(q) q(1:2) + e*[cos(q(3)); sin(q(3))];
    %System.Psi = @(q) [eye(2); [-e*sin(q(3)) e*cos(q(3))]];
    System.a = @(q) q(1:2);
    System.Psi = @(q) eye(2);
    System.dVs = @(q) [0; 0];%[5*(q(1) - 2); 5*(q(2) - 3); 0];

    %System.nPsi = @(q) System.annPsi(q)'*System.annPsi(q);
    
    %% R-passivity components
    System.lambda = lambda; 
    %System.dMinvdt = @(q,qdot)zeros(3,3);
    %System.qdotM = @(q,qdot) zeros(3, 3);
    % System r-passivity
    System.r = @(q, qdot) System.Psi(q)'*qdot + System.lambda*System.a(q);
    System.R = @(q, r) 0.5*r'*r;
    System.gamma = 1;

    System.Fd = @(q) System.Psi(q);%System.S(q)'*pinv(System.Psi(q)');
    System.gamma = 1;
    System.Kc = @(q, qdot) System.dPsi(q, qdot)' + ...
        System.Psi(q)'*(System.lambda + System.gamma);
    
    % Local Damping
    System.Kv = @(q) eye(2);

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
    SInfo.n = 2;
    SInfo.name = 'Differential Drive';
    SInfo.identifier = [SInfo.name ' #' num2str(index)];
    SInfo.legend = {[SInfo.name ' x'], [SInfo.name ' y'], [SInfo.name ' \theta']};
    SInfo.plotf = @plotDrive;
    SInfo.filename = filename;
    SInfo.h = 0.1;
    SInfo.l = 0.2;
   
    save(filename, 'System');
    fprintf(['    [S' num2str(index) '] Drive System Added\n']);
end

    %% IDA-PBC
    %System.a = @(q) q(1:2) - 1/g3 * [k3*sin(q(3)); (k3-k1*e)*(1-cos(q(3)))];
    %System.Psi = @(q) [eye(2); [-k3/g3*cos(q(3)) -(k3-k1*e)/g3*sin(q(3))]];
    %System.dPsi = @(q, qdot) [zeros(2, 2); [k3/g3*sin(q(3))*qdot(3) -(k3-k1*e)/g3*cos(q(3))*qdot(3)]];
    
    % Control
%     System.Md = @(q) [k1*e*(cos(q(3)))^2 + k3 k1*e*cos(q(3))*sin(q(3)) k1*cos(q(3));...
%                       k1*e*cos(q(3))*sin(q(3)) -k1*e*(cos(q(3)))^2+k3 k1*sin(q(3));...
%                       k1*cos(q(3)) k1*sin(q(3)) k2];
%     Find_dMdq;
%     System.dMd_dq = dMd_dq;   
% 
%     J1 = @(q, p) p'*inv(System.Md(q)) * [-2*e*cos(q(3)); 2*e*sin(q(3)); 1];
%     J2 = @(q, p) p'*inv(System.Md(q)) * [0; 1; 0];
%     J3 = @(q, p) p'*inv(System.Md(q)) * [-1; 0; 0];
%     System.J = @(q, p) -k1*g3/2*[0 J1(q, p) J2(q, p); -J1(q, p) 0 J3(q, p); -J2(q, p) -J3(q, p) 0];
%     System.dVs = @(q) [0; 0; g/g3*sin(q(3))];
    %System.Kv = @(q, p)eye(2);
    %System.Phi = @(q) (System.F(q)'*System.F(q))*System.F(q)'*System.Md(q)*inv(System.M(q))*System.Psi(q);
    
    
    %% Fperp = Psiperp
%     System.Ml = @(q) [1 0 l*cos(q(3)); 0 1 l*sin(q(3)); 0 0 1/e*l-(a*e)/(1+e^2)];
%     det(System.Ml([1; 2; 3]))
%     System.Psi = @(q) [eye(2); [e*cos(q(3)) e*sin(q(3))]];
%     System.a = @(q) q(1:2) + [e*sin(q(3)); e*(1-cos(q(3)))];
%     System.dPsi = @(q, qdot) [zeros(2,2); [-e*sin(q(3))*qdot(3) e*cos(q(3))*qdot(3)]];
%     System.annPsi = @(q) [1/e*cos(q(3)) 1/e*sin(q(3)) -1];
    