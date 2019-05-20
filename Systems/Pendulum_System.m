function [System, SInfo] = Pendulum_System(lambda, index, varargin)
    k_1 = 1;
    l = 1;
    
    filename = ['Systems/Pendulum_n' num2str(index)];
    
    System.M = 1;
    System.dV = @(q) -sin(q);
    
    System.Md = 1;
    System.dVs = @(q) sin(q) + k_1*q;
    System.Kv = 2;
    System.a = @(q) [sin(q)*l, cos(q)*l];
    System.lambda = lambda;
    save(filename, 'System');
    
    %% Save system information
    SInfo.n = 1;
    SInfo.l = l;
    SInfo.name = 'Pendulum';
    SInfo.identifier = ['Pendulum #' num2str(index)];
    SInfo.legend = {[SInfo.name ' q1']};
    SInfo.plotf = @PlotPendulum;
    SInfo.filename = filename;
end
