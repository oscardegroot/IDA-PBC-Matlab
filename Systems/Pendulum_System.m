function [System, SInfo] = Pendulum_System(lambda, index, varargin)
    k_1 = 1;
    l = 1;
    syms q
    filename = ['Systems/Pendulum_n' num2str(index)];
    
    M = 1 + 0*q;
    dV = -sin(q);
    
    %Md = 1;
    dVs = sin(q) + k_1*q;
    Kv = 0.5 + 0*q;%2
    z = [sin(q)*l, cos(q)*l];
    System.lambda = lambda;
    
    System.M = saveFcnHandle1D(M, q, index);
    System.z = saveFcnHandle1D(z, q, index, 'a');
    System.dV = saveFcnHandle1D(dV, q, index);
    System.dVs = saveFcnHandle1D(dVs, q, index);
    System.Kv = saveFcnHandle1D(Kv, q, index);

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
