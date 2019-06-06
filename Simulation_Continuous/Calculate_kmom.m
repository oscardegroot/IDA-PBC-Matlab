function [y] = Calculate_kmom(m, mbar)

    if(m >= mbar)
        y = 0;
    elseif(m < mbar/2)
        y = 1;
    else
        y = 4*(4*m^3 - 9*m^2*mbar+6*mbar^2*m-mbar^3)/(mbar^3);
    end
end