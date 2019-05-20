function [drp] = getMom(System, q, dr)

    mbar = System.mbar;

    %% Calculate Mom
    S = svd(System.Psi(q));
    mom = 1;
    for i = 1 : numel(S)
        mom = mom*abs(S(i));
    end
    
    drp = dr;
    
    % If outside of the range of mbar
    if(mom > mbar)
        return
    end

    %% Calculate the surface and project dr
    if(mom <= mbar)
        dmomsurface = mom*System.Psimom(q)'*pinv(System.Psi(q)');
        nm = dmomsurface'/norm(dmomsurface);
        
        % Only act if the dot product is negative
        if(dr'*nm >= 0)
            return
        end
        
        if(mom > mbar/2)
            drp = dr - (dr'*nm)*nm*System.kmom(mom, mbar);
        else
            drp = dr - (dr'*nm)*nm;
        end
    end
end