function [drp] = getMom(System, q, dr)

    mbar = System.mbar;

    %% Calculate Mom
    S = svd(System.Psi(q));
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
        dmomsurface = mom*System.Psimom(q)'*pinv(System.Psi(q)');
        nm = dmomsurface'/norm(dmomsurface);
        
        % Add the push force
        drp = dr + System.push_gain*System.kmom(mom, mbar/2)*nm;
        
        % Only act if the dot product is negative
        if(dr'*nm >= 0)
            return
        end
        
        % Add the pull force
        drp = drp - (dr'*nm)*nm*System.kmom(mom, mbar);
    end
end