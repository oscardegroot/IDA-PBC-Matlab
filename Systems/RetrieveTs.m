function Ts = RetrieveTs(var) 
% Set the sampling time if given (discrete)
    if(size(var) > 1)
        if(var{2} > 0)
            Ts = var{2};
        else
            error('Ts <= 0');
        end
    % Otherwise the system is continuous
    else
        Ts = 0;
    end
end