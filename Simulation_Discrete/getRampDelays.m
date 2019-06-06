function [delays] = getRampDelays(length, Ts, Tdot, d_min, d_max)
    
    % Convert delay timings to sampling frequency equivalences
    d_min = ceil(d_min / Ts);
    d_max = floor(d_max / Ts);
    
    delays(1) = d_min;
    for i = 2:length
        % Ramp
        delays(i) = delays(i-1) + Tdot;
        
        % Clip
        delays(i) = min(max(delays(i), d_min), d_max);
    end
    
    delays = ceil(delays); % Delays less than Ts are Ts

end