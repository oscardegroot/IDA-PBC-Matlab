function Ts = RetrieveTs(var) 
% Set the sampling time if given (discrete)
    if(size(var) > 0)
        if(var{1} > 0)
            Ts = var{1};
        else
            error('Ts <= 0');
        end
    % Otherwise the system is continuous
    else
        Ts = 0;
    end
end