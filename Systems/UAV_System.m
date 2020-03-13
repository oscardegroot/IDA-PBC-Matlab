function [System, SInfo] = UAV_System(lambda, index, varargin)

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
    System.Minv = @(q) eye(3);
    System.n = 3;
    System.name = 'UAV';
    System.dV = @(q) [0;0;-g/e*sin(q(3))];
    System.F = @(q) [eye(2); [1/e*cos(q(3)) 1/e*sin(q(3))]];
    System.Fp = @(q) [cos(q(3)) sin(q(3)) -e];
    System.dHdq = @(q) System.dV(q);
    
    %% IDA-PBC
    System.a = @(q) q(1:2) - 1/g3 * [k3*sin(q(3)); (k3-k1*e)*(1-cos(q(3)))];
    System.Psi = @(q) [eye(2); [-k3/g3*cos(q(3)) -(k3-k1*e)/g3*sin(q(3))]];
    System.dPsi = @(q, qdot) [zeros(2, 2); [k3/g3*sin(q(3))*qdot(3) -(k3-k1*e)/g3*cos(q(3))*qdot(3)]];
    
    %Control
    System.Md = @(q) [k1*e*(cos(q(3)))^2 + k3 k1*e*cos(q(3))*sin(q(3)) k1*cos(q(3));...
                      k1*e*cos(q(3))*sin(q(3)) -k1*e*(cos(q(3)))^2+k3 k1*sin(q(3));...
                      k1*cos(q(3)) k1*sin(q(3)) k2];
    Find_dMdq;
    System.dMd_dq = dMd_dq;   

    J1 = @(q, p) p'*inv(System.Md(q)) * [-2*e*cos(q(3)); 2*e*sin(q(3)); 1];
    J2 = @(q, p) p'*inv(System.Md(q)) * [0; 1; 0];
    J3 = @(q, p) p'*inv(System.Md(q)) * [-1; 0; 0];
    System.J = @(q, p) -k1*g3/2*[0 J1(q, p) J2(q, p); -J1(q, p) 0 J3(q, p); -J2(q, p) -J3(q, p) 0];
    System.dVs = @(q) [0; 0; g/g3*sin(q(3))];
    System.Kv = @(q) zeros(2);%pinv(System.F(q))*System.Md(q)*pinv(System.F(q)');
    System.Fd = @(q) System.Md(q)*System.Psi(q)*inv(System.Psi(q)'*System.Psi(q));

    %System.Phi = @(q) (System.F(q)'*System.F(q))*System.F(q)'*System.Md(q)*inv(System.M(q))*System.Psi(q);
    
    
    %% No null space, but Md
%     System.a = @(q) q(1:2) - 1/g3 * [k3*sin(q(3)); (k3-k1*e)*(1-cos(q(3)))];
%     System.Psi = @(q) [eye(2); [-k3/g3*cos(q(3)) -(k3-k1*e)/g3*sin(q(3))]];
%     System.annPsi = @(q) [k3/g3*cos(q(3)) (k3-k1*e)/g3*sin(q(3)) 1];
%     System.dPsi = @(q, qdot) [zeros(2, 2); [k3/g3*sin(q(3))*qdot(3) -(k3-k1*e)/g3*cos(q(3))*qdot(3)]];
%     System.Md = @(q) [k1*e*(cos(q(3)))^2 + k3 k1*e*cos(q(3))*sin(q(3)) k1*cos(q(3));...
%                       k1*e*cos(q(3))*sin(q(3)) -k1*e*(cos(q(3)))^2+k3 k1*sin(q(3));...
%                       k1*cos(q(3)) k1*sin(q(3)) k2];
%     System.dVs = @(q) [0; 0; g/g3*sin(q(3))];
    %% Mapped Potential
%     a = 0.45;
%     k3 = (a/e + a/e*sqrt(1 - 4*e/a))/2;
%     System.a = @(q) q(1:2) + [-a/k3*sin(q(3)); a/k3*(cos(q(3)) - 1)];
%     %System.t = @(q) a*[cos(q(3)) sin(q(3))]*q(1:2) + k3*q(3);
%     System.Psi = @(q) [eye(2); [-a/k3*cos(q(3)) -a/k3*sin(q(3))]];
%     System.dPsi = @(q, qdot) [zeros(2,2); [a/k3*sin(q(3))*qdot(3) -a/k3*cos(q(3))*qdot(3)]];
%     System.annPsi = @(q) [a*cos(q(3)) a*sin(q(3)) k3];
%     System.anndPsi = @(q, qdot) [-a*sin(q(3))*qdot(3) a*cos(q(3))*qdot(3) 0];
%     System.dVs = @(q) [0; 0; (g/a)*sin(q(3))];
    %System.Md = @(q) diag([1 1 -k3/(e*a)]);
    %b1 = 4.06; b2 = 1; b3 = 1;
    %System.Md = @(q) [b1*e*(cos(q(3)))^2 + b3 b1*e*cos(q(3))*sin(q(3)) b1*cos(q(3));...
    %              b1*e*cos(q(3))*sin(q(3)) -b1*e*(cos(q(3)))^2+b3 b1*sin(q(3));...
    %              b1*cos(q(3)) b1*sin(q(3)) b2];
    %DeriveMd;
    %System.dq3dt = @(q) 5*[0 0 1/(System.annPsi(q)*[0; 0; 1])];
    %System.dVs = @(q) k3*sin(System.t(q));%[0; 0; -System.t(q)];
% 
    System.nPsi = @(q) System.annPsi(q)'*System.annPsi(q);
%     %System.Ft = @(q) (eye(3) - pinv(System.Psi(q)')*System.Psi(q)')*System.annPsi(q)'*inv(System.annPsi(q)*(eye(3) - pinv(System.Psi(q)')*System.Psi(q)')*System.annPsi(q)');
%     System.Ft = @(q) System.annPsi(q)'*inv(System.annPsi(q)*System.annPsi(q)');
%     System.Md = @(q) eye(3);
    %% R-passivity components
    System.lambda = lambda; 
    System.dMinvdt = @(q,qdot)zeros(3,3);
    System.qdotM = @(q,qdot) zeros(3, 3);
    % System r-passivity
    System.r = @(q, qdot) System.Psi(q)'*qdot + System.lambda*System.a(q);
    System.R = @(q, r) 0.5*r'*r;
    System.gamma = 1;
    %% Derived Ml
    %l = 2; % l < 1
    % Scale determinant to 1?
    %System.Md = @(q) System.Ml(q);
    %System.Md = @(q) [a/k3 0 l*cos(q(3)); 0 a/k3 l*sin(q(3)); 0  0 (l-1)/e];
    %System.Fd = @(q) System.Md(q)*System.Psi(q)*inv(...
    %   System.Psi(q)'*System.Md(q)*System.Psi(q)); % Matching holds, but doesn't work
    
    System.gamma = 1;
    System.Kc = @(q, qdot) System.dPsi(q, qdot)' + ...
        System.Psi(q)'*(System.lambda + System.gamma);
    
    % Local Damping
    %System.Kv = @(q) eye(2);

    %% Discrete time compensation
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
    SInfo.identifier = [SInfo.name ' #' num2str(index)];
    SInfo.legend = {[SInfo.name ' x'], [SInfo.name ' y'], [SInfo.name ' \theta']};
    SInfo.plotf = @PlotUAV;
    SInfo.filename = filename;
    SInfo.h = e;
    SInfo.l = e*2;
   
    save(filename, 'System');
    fprintf(['    [S' num2str(index) '] UAV System Added\n']);
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
    