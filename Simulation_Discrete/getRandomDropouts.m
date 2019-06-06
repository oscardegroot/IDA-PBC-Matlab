%% Get random dropouts
% p_drop: drop chance
function [dropouts] = getRandomDropouts(length, p_drop)

    % Generate a random sequence
    dropouts = rand(length, 1);
    
    % Set all values larger than p_drop to no dropout (0)
    dropouts(dropouts < (1 - p_drop)) = 0;
    
    % Set the rest to dropout (1)
    dropouts(dropouts ~= 0) = 1;
end