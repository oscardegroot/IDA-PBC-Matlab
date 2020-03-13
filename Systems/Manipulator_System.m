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
    
    full_calculations = true;
    filename = ['Systems/Manipulator' num2str(n_link) '_n' num2str(index)];
    
    if(isfield(Parameters, 'location'))
        location = Parameters.location;
    else
        location = zeros(3, 1);
    end
        
    % Initialise symbolics
    q = sym('q', [n_link, 1], 'real'); p = sym('p', [n_link, 1], 'real');
    qdot = sym('qdot', [n_link, 1], 'real');
    assume([q p qdot], 'real');
    
    if(full_calculations)
        

        %% Construct a mass matrix
        % Based on convergence from energy coordinates to generalised
        % coordinates (dx/dq'*Mx*dx/dq)
        % Define Mx
        Mw = diag(reshape([I M M]', [n_link*3, 1]));

        Partial_wq = [];

        %assume([q1, q2, q3], 'real');

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
        assume(Mm, 'real');
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

        nPsi = (eye(3) - pinv(Psi')*Psi');%null(Psi')';
        dnPsi = ddt(nPsi);

        % Calculate dmomdq derivatives
        Psi_1 = diff(Psi', q(1));Psi_2 = diff(Psi', q(2));Psi_3 = diff(Psi', q(3));

        %% Process matrices
        Mm = Mm';
        dMdt = ddt(Mm);

        dHdq = dV + 0.5*qdotM*qdot;
        r = Psi'*qdot + lambda*z;
        
         %Define dmomdq
        Psimom = [trace(Psi_1*pinv(Psi')); ...
            trace(Psi_2*pinv(Psi')); trace(Psi_3*pinv(Psi'))];
        
         %% Convertions to matlabfunctions
        System.M = saveFcnHandle(Mm, q, index);
        System.z = saveFcnHandle(z, q, index, 'a');
        System.Psi = saveFcnHandle(Psi, q, index);
        System.nPsi = saveFcnHandle(nPsi, q, index);
        System.Mdot = saveFcnHandle(dMdt, q, index, 'Mdot', qdot);
        System.dHdq = saveFcnHandle(dHdq, q, index, 'dHdq', qdot);
        System.dPsi = saveFcnHandle(dPsi, q, index, 'dPsi', qdot);
        %System.dnPsi = saveFcnHandle(dnPsi, q, index, 'dnPsi', qdot);
        System.lambda = lambda;
        System.r = saveFcnHandle(r, q, index, 'r', qdot);
        System.Psimom = saveFcnHandle(Psimom, q, index); 
    end
    
    %% Calculations that are always performed!
    

    % Goal tracking
    if(index == 1)
        gain = 4;%30;
        goal = -pi/4;
        dVs = [-gain*(goal - q(1)); 0; 0];
        ddVs = [gain 0 0; 0 0 0; 0 0 0] + q'* zeros(3,3) * q;
    end
    % Manipulability (det(JJ^T) = Vs)
%     gain = 5;
%     Vs = -gain*det(Psi'*Psi);
%     dVs = -[diff(Vs, q(1)); diff(Vs, q(2)); diff(Vs, q(3))];
%     %dVs = subs(dVs,{q1, q2, q3},{'q1', 'q2', 'q3'});
%     ddVs = [diff(dVs, q(1)), diff(dVs, q(2)), diff(dVs, q(3))];

    % No dVs
    if(index == 2)
        dVs = zeros(3, 1).*q;
        ddVs = 0*q*q';
    end
    

    System.dVs = saveFcnHandle(dVs, q, index);
    System.ddVs = saveFcnHandle(ddVs, q, index);
   

    %% Singularity Avoidance
    %System.mbar = mbar;
    %System.push_gain = 5;
    % The function that maps k smoothly
    %System.kmom = @(m, mbar) Calculate_kmom(m, mbar);

    % DAMPING TO PROVE CONDITION
   % System.Kv = @(q, qdot) System.Minv(q)*System.dMinvdt(q, qdot)*System.M(q) + System.Minv(q);

    %% Discrete Parameters
    System.Ts = RetrieveTs(varargin);
    
    %% Save system information
    SInfo.n = n_link;
    SInfo.name = 'Manipulator';
    SInfo.identifier = ['Manipulator #' num2str(index)];
    SInfo.location = location;
    SInfo.plotf = @PlotManipulator;
    SInfo.filename = filename;
    
    fprintf(['    [S' num2str(index) '] Manipulator System Added\n']);
    save(['Systems/System' num2str(index)], 'System', 'SInfo');
end

%     Mm = matlabFunction(Mm');
%     Mm = @(q) Mm(q(1), q(2), q(3));
%     Mminv = matlabFunction(Mminv);
%     Mminv = @(q) Mminv(q(1), q(2), q(3));
%     dV = matlabFunction(dV);
%     dV = @(q) dV(q(1), q(2), q(3));
%     z = matlabFunction(z);
%     z = @(q) z(q(1), q(2), q(3));
%     qdotM = matlabFunction(qdotM);
%     System.qdotM = @(q, qdot) qdotM(q(1), q(2), q(3), qdot(1), qdot(2), qdot(3));
%     Psi = matlabFunction(Psi);
%     Psi = @(q) Psi(q(1), q(2), q(3));
%     nPsi = matlabFunction(nPsi);
%     nPsi = @(q) nPsi(q(1), q(2), q(3));

%% Disabled?
%     Psi_1 = matlabFunction(Psi_1);
%     Psi_1 = @(q) Psi_1(q(1), q(2), q(3));
%     Psi_2 = matlabFunction(Psi_2);
%     Psi_2 = @(q) Psi_2(q(1), q(2), q(3));
%     Psi_3 = matlabFunction(Psi_3);
%     Psi_3 = @(q) Psi_3(q(1), q(2), q(3));
    
    %% System Matrices
%     System.M = @(q) Mm(q);
%     System.Minv = @(q) Mminv(q);
%     System.dMdt = @(q, qdot) dMdt(q, qdot);
%     System.dMinvdt = @(q, qdot) dMinvdt(q, qdot);
%     System.dV = @(q) dV(q);
%     System.dHdq = @(q, qdot) System.dV(q) + 0.5*System.qdotM(q, qdot)*qdot;
%     System.F = @(q) eye(3);
%     System.a = @(q) z(q);
%     System.Psi = @(q) Psi(q);
%     System.dPsi = @(q, qdot) dPsi(q, qdot);
%     System.nPsi = @(q) nPsi(q);
%     System.dnPsi = @(q, qdot) dnPsi(q, qdot);

 %% Control Parameters
%     if(isfield(Parameters, 'dVs'))
%         System.dVs = Parameters.dVs;
%     else
%         System.dVs = @(q) [0];
%     end