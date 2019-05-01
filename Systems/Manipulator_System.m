function [System, SInfo] = Manipulator_System(lambda, epsilon, location, index, varargin)
    g = 9.81;
    n_link = 3;
    L = [0.5; 0.5; 0.5];
    M = [2; 2; 2];
    I = [0.05; 0.05; 0.05]; 
    
    algorithm = 1;
    filename = ['Systems/Manipulator' num2str(n_link) '_n' num2str(index)];

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
    temp_mat = qdot'*Mm*qdot;
    temp_qdotM = qdot'*Mm;
    
    % Construct d/dq
    dMdq = []; dMdt = []; qdotM = [];
    for i = 1 : n_link
         dMdq = [dMdq diff(temp_mat, q(i))];
         qdotM = [qdotM; diff(temp_qdotM, q(i))];
    end
    
    %% Calculate the potential and cooperative coordinates
    dV = [0; 0; 0];
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
    
    if(algorithm == 1)
        dPsi = ddt(Psi);
    else
        dPPsi = ddt(inv(Psi'*Psi + epsilon * eye(2)) * Psi');
    end
    
    %% Calculate misc matrices
    dMdt = ddt(Mm);
    Mminv = inv(Mm');
    dMinvdt = ddt(Mminv);
    Find_drLr;
    
    %% Convertions to matlabfunctions
    Mm = matlabFunction(Mm');
    Mm = @(q) Mm(q(1), q(2), q(3));
    Mminv = matlabFunction(Mminv);
    Mminv = @(q) Mminv(q(1), q(2), q(3));
    dV = matlabFunction(dV);
    dV = @(q) dV(q(1), q(2), q(3));
    z = matlabFunction(z);
    z = @(q) z(q(1), q(2), q(3));
    Psi = matlabFunction(Psi);
    Psi = @(q) Psi(q(1), q(2), q(3));
    
    dMdq = matlabFunction(dMdq');
    dMdq = @(q, qdot) dMdq(q(1), q(2), q(3), qdot(1), qdot(2), qdot(3));
    qdotM = matlabFunction(qdotM);
    System.qdotM = @(q, qdot) qdotM(q(1), q(2), q(3), qdot(1), qdot(2), qdot(3));
    
    %% Define the system structure
    System.M = @(q) Mm(q);
    System.Minv = @(q) Mminv(q);
    System.dMdq = @(q, qdot) dMdq(q, qdot);
    System.dMdt = @(q, qdot) dMdt(q, qdot);
    System.dMinvdt = @(q, qdot) dMinvdt(q, qdot);
    System.dV = @(q) dV(q);
    System.F = @(q) eye(3);
    System.a = @(q) z(q);
    System.Psi = @(q) Psi(q);
    
    if(algorithm == 1)
        System.dPsi = @(q, qdot) dPsi(q, qdot);
    else
        System.dPPsi = @(q, qdot) dPPsi(q, qdot);
    end
    System.drLr = @(q, qdot) drLr(q, qdot);
    
    % Control
    System.dVs = @(q) [0; 0; 0];%
    %Vs = @(q)4*q(2)^2;
    System.Phi = @(q) System.Psi(q);
    
    % r-passivity specific variables
    System.lambda = lambda;
    System.epsilon = epsilon;
    
    %% Kv for r with psi transposed
    if(algorithm == 1)
       % -> Kv modifies the "matching" condition while tau does not.
       % gain Here helps ONLY in thediscrete case!!!0.1*
       %System.r = @(q, qdot) System.Psi(q)'*qdot + System.lambda*System.a(q);
       System.r = @(q, qdot) System.Psi(q)'*System.M(q)*qdot + System.lambda*System.a(q);
       System.Kv = @(q, qdot) eye(3);%lambda*System.Psi(q)*System.Psi(q)';% + eye(3);%lambda*eye(3);%-0.5*System.qdotM(q, qdot);% + eye(3);
       System.Linv = @(q) inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2));
       %System.Sigma = @(q) svd(System.Psi(q));
%        System.Pcomp = @(q) inv(eye(3) - System.Psi(q)*System.Linv(q)*System.Psi(q)'*...
%            (System.M(q) - eye(3))*System.Minv(q));
       %System.dzLz = @(q, qdot) 0.5*dzLz(q, qdot);
       System.R = @(q, r) 0.5*r'*System.Linv(q)*r;% + 0.5*lambda*System.a(q)'*System.a(q);
       %System.R = @(q, r) 0.5*r'*inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2))*r;
              %Hd = @(q, p) 0.5*p'*System.M(q)*p + Vs(q);
%        dL = ddt(inv(System.Psi(q)'*System.Psi(q) + System.epsilon*eye(2)));
%        System.dVzdq = @(z, q, qdot) 0.5*lambda*z'*dL(q, qdot)*z;
    end
    %% Kv for r with psi pseudo
    if(algorithm == 2)
        System.Kv = @(q, qdot) lambda* System.Psi(q)*System.Psi(q)' - 0.5*System.dMdt(q, qdot)+...
        System.Psi(q)*System.dPPsi(q, qdot)*System.M(q);
        System.R = @(q, r) 0.5*r'*r;
    end

    %% Discrete Data
    System.Ts = RetrieveTs(varargin);
    
    % Discrete time compensation
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

    save(filename, 'System');
    
    %% Save system information
    SInfo.n = size(Mm(q), 1);
    SInfo.name = 'Manipulator';
    SInfo.legend = {[SInfo.name ' q1'], [SInfo.name ' q2'], [SInfo.name ' q3']};
    SInfo.location = location;
    SInfo.plotf = @PlotManipulator;
    SInfo.filename = filename;
end