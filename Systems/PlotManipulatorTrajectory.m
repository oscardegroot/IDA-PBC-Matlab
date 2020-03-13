function PlotManipulatorTrajectory(y, Simulation, index)
    
    
    for i = 1 : size(y, 1)
        z(i, :) = feval(['a' num2str(index)], y(i, :)');
    end
    
    plot(z(:, 1), z(:, 2), [Simulation.colors{index} '--'], 'LineWidth', 1.5);

    %% Arrow stuff...
%     scale = 20.0;
%     arrow_frequency = 30;
%     
%     has_been_big = false;
%     some_threshold = 0.002;
%     N_arrows = floor(size(y, 1) / arrow_frequency);
%     x = []; y = []; xd = []; yd = [];
%     for i = 1 : N_arrows
%         z_arrow = z((i - 1) * arrow_frequency + 1, :);
%         z_diff_arrow = z((i - 1) * arrow_frequency + 2, :);
%         zdiff = z_diff_arrow - z_arrow;
%         
%         if(norm(zdiff) < some_threshold)
%             if(has_been_big)
%                 break;
%             end
%         else
%             has_been_big = true;
%         end
%         
%         %zdiff = zdiff ./ norm(zdiff);
%         
%         x = [x; z_arrow(1)];
%         y = [y; z_arrow(2)];
%         xd = [xd; zdiff(1)]; 
%         yd = [yd; zdiff(2)]; 
%     end
%     
%     quiver(x, y, scale* xd, scale * yd,0, 'k-', 'LineWidth', 2.5);
%     nicenorm = sqrt(downsample(z_diff(:, 1).^2 + z_diff(:, 2).^2, arrow_frequency));
%     nicenorm(nicenorm == 0) = 1;
%     quiver(downsample(z(1:end-arrow_frequency, 1), arrow_frequency),...
%         downsample(z(1:end-arrow_frequency, 2), arrow_frequency),...
%         scale*downsample(z_diff(:, 1), arrow_frequency)./nicenorm , ...
%         scale*downsample(z_diff(:, 2), arrow_frequency)./ nicenorm ,0,...
%         ['k-'], 'LineWidth', 2.0);

    
    % 'Marker', '>', 'MarkerIndices', [300, 400],
    % Formation...
%     z_e = [z(end, 1); z(end, 2)];
%     if(index == 1)
%         z_f = z_e + Simulation.Formation_Goal';
%     else
%         z_f = z_e - Simulation.Formation_Goal';
%     end
%    plot([z_e(1); z_f(1)], [z_e(2); z_f(2)], [Simulation.colors{index} 'o--']);

end