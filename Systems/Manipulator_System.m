%% Varargin: Ts
%% Parameters:
%   Location: 3D mounting location
%   Vs: Potential function (@(q))
%
function [System, SInfo] = Manipulator_System(lambda, mbar, Parameters, index, varargin)
    g = 9.81;
    n_link = 3;
    L = 0.5*ones(n_link,1);
    M = 2*ones(n_link,1);
    I = 0.05*ones(n_link,1);
    
    filename = ['Systems/Manipulator' num2str(n_link) '_n' num2str(index)];
    
    if(isfield(Parameters, 'location'))
        location = Parameters.location;
    else
        location = zeros(3, 1);
    end
    
    %% Construct a mass matrix
    % Based on convergence from energy coordinates to generalised
    % coordinates (dx/dq'*Mx*dx/dq)
    % Define Mx
    Mw = diag(reshape([I M M]', [n_link*3, 1]));
    
    Partial_wq = [];
    q = sym('q', [n_link, 1]); p = sym('p', [n_link, 1]);
    qdot = sym('qdot', [n_link, 1]);
    assume([q p qdot], 'real');
    
    % Move per column
    for i = 1 : n_link
        
        v = [];
        dx = 0; dy = 0;
        gamma =  0;
        % Move per n_link rows
        for j = 1 : n_link
            if(j < i)
                v = [v; zeros(n_link, 1)];
            else
                % Set the joint angle
                gamma =  gamma + q(j);

                % Calculate the increase in x per column
                dgamma =  1;
                dx = dx + 0.5*L(j)*cos(gamma);
                dy = dy - 0.5*L(j)*sin(gamma);
                
                
                v = [v;dgamma; dx; dy];
                dx = dx + 0.5*L(j)*cos(gamma);
                dy = dy - 0.5*L(j)*sin(gamma);
            end
        end
        Partial_wq = [Partial_wq v];
    end
    Partial_wq = simplify(Partial_wq);
    
    % Calculate the mass matrix
    Mm = Partial_wq' * Mw * Partial_wq;
    Mm = simplify(Mm);
    
    %% Calculate other matrices such as qdot'*dM/dq
    temp_qdotM = qdot'*Mm;
    
    % Construct d/dq
    qdotM = [];
    for i = 1 : n_link
         qdotM = [qdotM; diff(temp_qdotM, q(i))];
    end
    
    %% Calculate the potential and cooperative coordinates
    dV = zeros(n_link, 1);
    x = 0; y = 0; gamma = 0;
    for i = 1 : n_link
        
        % Potential
        v = g*M(i)*Partial_wq((i-1)*n_link + 3, :)';
        dV = dV + v;
        
        % Cooperative
        gamma = gamma + q(i);
        x = x + L(i)*sin(gamma);
        y = y + L(i)*cos(gamma);
    end
    
    z = simplify([x; y]) + location(1:2);
    dV = simplify(dV);
    
    %% Construct dz/dq (=psi) and dpsi/dt
    Psi = [];  
    for i = 1 : n_link
         Psi = [Psi diff(z, q(i))];
    end
    Psi = simplify(Psi');
    dPsi = ddt(Psi);
    
    % Calculate dmomdq derivatives
    Psi_1 = diff(Psi', q(1));Psi_2 = diff(Psi', q(2));Psi_3 = diff(Psi', q(3));
    
    %% Calculate misc matrices
    dMdt = ddt(Mm);
    Mminv = inv(Mm');
    dMinvdt = ddt(Mminv);
    %Find_drLr;
    
    %% Convertions to matlabfunctions
    Mm = matlabFunction(Mm');
    Mm = @(q) Mm(q(1), q(2), q(3));
    Mminv = matlabFunction(Mminv);
    Mminv = @(q) Mminv(q(1), q(2), q(3));
    dV = matlabFunction(dV);
    dV = @(q) dV(q(1), q(2), q(3));
    z = matlabFunction(z);
    z = @(q) z(q(1), q(2), q(3));
    qdotM = matlabFunction(qdotM);
    System.qdotM = @(q, qdot) qdotM(q(1), q(2), q(3), qdot(1), qdot(2), qdot(3));
    Psi = matlabFunction(Psi);
    Psi = @(q) Psi(q(1), q(2), q(3));
    Psi_1 = matlabFunction(Psi_1);
    Psi_1 = @(q) Psi_1(q(1), q(2), q(3));
    Psi_2 = matlabFunction(Psi_2);
    Psi_2 = @(q) Psi_2(q(1), q(2), q(3));
    Psi_3 = matlabFunction(Psi_3);
    Psi_3 = @(q) Psi_3(q(1), q(2), q(3));
    
    %% System Matrices
    System.M = @(q) Mm(q);
    System.Minv = @(q) Mminv(q);
    System.dMdt = @(q, qdot) dMdt(q, qdot);
    System.dMinvdt = @(q, qdot) dMinvdt(q, qdot);
    System.dV = @(q) dV(q);
    System.dHdq = @(q, qdot) System.dV(q) + 0.5*System.qdotM(q, qdot)*qdot;
    System.F = @(q) eye(3);
    System.a = @(q) z(q);
    System.Psi = @(q) Psi(q);
    System.dPsi = @(q, qdot) dPsi(q, qdot);
    
    %% Control Parameters
    if(isfield(Parameters, 'dVs'))
        System.dVs = Parameters.dVs;
    else
        System.dVs = @(q) [0;0;0];
    end
    
    System.nPsi = @(q) eye(3) - pinv(System.Psi(q))'*System.Psi(q)';
    System.Kv = @(q, qdot) eye(3);

    %% r-passivity parameters (z_dot based)
    System.lambda = lambda;
    System.r = @(q, qdot) System.Psi(q)'*qdot + System.lambda*System.a(q);
    System.rdot = @(q, qdot, p, pdot)  System.Psi(q)'*System.Minv(q)*pdot + ...
       System.Psi(q)'*System.dMinvdt(q, qdot)*p + System.dPsi(q, qdot)'*qdot + ...
       System.Psi(q)'*System.lambda*qdot;
    System.R = @(q, r) 0.5*r'*r;
    
    %% Singularity Avoidance
    System.mbar = mbar;
    System.push_gain = 5;
    
    %Define dmomdq
    System.Psimom = @(q) [trace(Psi_1(q)*pinv(Psi(q)')); ...
        trace(Psi_2(q)*pinv(Psi(q)')); trace(Psi_3(q)*pinv(Psi(q)'))];
    
    % The function that maps k smoothly
    System.kmom = @(m, mbar) Calculate_kmom(m, mbar);
   
    %% Cooperative Matrices
    System.isLeader = false;
    
    %% Control Stash
    %System.S = @(q) System.Psi(q);
    %System.drLr = @(q, qdot) drLr(q, qdot);
    % Psi'p variables
    %System.r = @(q, qdot) System.S(q)'*System.M(q)*qdot + System.lambda*System.a(q);
    %System.rdot = @(q, qdot, p, pdot) System.S(q)'*pdot + ...
      % System.lambda*System.Psi(q)'*qdot;

    %% Discrete Parameters
    System.Ts = RetrieveTs(varargin);
    
    if(System.Ts > 0)
        %% Euler
        Lv = eye(3);
        kappa = 0;
        System.G = System.Ts*kappa*Lv;
        
        %% EMatching
        dV = ddt(dV(q));
        System.dVdt = @(q, qdot) dV(q, qdot);
        System.dVsdt = @(q, qdot) [0;0;0];
    end
    
    %% Save system information
    save(filename, 'System');
    SInfo.n = size(Mm(q), 1);
    SInfo.name = 'Manipulator';
    SInfo.identifier = ['Manipulator #' num2str(index)];
    SInfo.location = location;
    SInfo.plotf = @PlotManipulator;
    SInfo.filename = filename;
    
    fprintf(['    [S' num2str(index) '] Manipulator System Added\n']);
end