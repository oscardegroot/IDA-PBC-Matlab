function [System, SInfo] = Pointmass2D_System(lambda, epsilon, m, index, varargin)
    filename = ['Systems/2D_n' num2str(index)];
    
    System.Ts = RetrieveTs(varargin);
   
    % System
    System.M = @(q) m*eye(2);
    System.n = 2;
    System.name = '2D';
    System.dV = @(q) [0;m*9.81];
    System.F = @(q) eye(2);
    System.a = @(q) q;
    System.Psi = @(q) eye(2);
    
    % Control
    System.Md = @(q) m*eye(2);
    System.dVs = @(q) [0; 0];
    System.J = @(q) zeros(2,2);
    System.Kv = lambda*eye(2);
    System.lambda = lambda; % Local save of lambda
    System.Phi = @(q) System.Psi(q);
     
    % Discrete time compensation
    if(System.Ts > 0)
        Lv = eye(2);
        kappa = 0.1;
        System.G = kappa*Lv;
    end
    
    save(filename, 'System');
    
    SInfo.n = 2;
    SInfo.name = 'Point Mass';
    SInfo.legend = {[SInfo.name ' x'], [SInfo.name ' y']};
    SInfo.plotf = @PlotPoint;
    SInfo.filename = filename;
end