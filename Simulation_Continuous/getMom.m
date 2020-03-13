function [drp] = getMom(index, q, dr)

    index = num2str(index);
    mbar = 0.3;

    %% Calculate Mom
    S = svd(feval(['Psi' index], q));
    mom = 1;
    for i = 1 : numel(S)
        mom = mom*abs(S(i));
    end
    
    drp = dr;
    
    % Return if outside of the range of mbar
    if(mom > mbar)
        return
    end

    %% Calculate the surface and project dr
    if(mom <= mbar)
        dmomsurface = mom*feval(['Psimom' index], q)'*...
            pinv(feval(['Psi', index], q)');
        nm = dmomsurface'/norm(dmomsurface);
        
        % Add the push force
        drp = dr + 5.0*Calculate_kmom(mom, mbar/2)*nm;
        
        % Only act if the dot product is negative
        if(dr'*nm >= 0)
            return
        end
        
        % Add the pull force
        drp = drp - (dr'*nm)*nm*Calculate_kmom(mom, mbar/2);
    end
end