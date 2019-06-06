%% Get a random walk series for delays Note:
% Tdot in (0, 1) -> % of Ts
% dmax in s
% dmin in s

function [delays] = getRandomDelays(length, Ts, Tdot, d_min, d_max)

    % Convert delay timings to sampling frequency equivalences
    d_min = ceil(d_min / Ts);
    d_max = floor(d_max / Ts);

    % Initialise the delay
    delays(1) = rand()*(d_max - d_min)+d_min;
    
    for i = 2 : length
        % Walk
        delays(i) = delays(i - 1) + (-Tdot + rand()*2*Tdot);
        
        % Clip
        delays(i) = min(max(delays(i), d_min), d_max);
    end
    
    delays = ceil(delays); % Delays less than Ts are Ts

end