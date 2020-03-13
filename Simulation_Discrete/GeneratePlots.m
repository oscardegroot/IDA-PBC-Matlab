%% Generate plots for a simulation setup %%
SetLegend;

%% Plot
if(Simulation.plots)
    %% Cooperative coordinate plot
    figure;
    % Plot system 1, x and y
    hold on;
    plot(z1.Time, z1.Data(:, 1), [Simulation.colors{1} '-'], 'Linewidth', 1.5);
    plot(z1.Time, z1.Data(:, 2), [Simulation.colors{2} '-'], 'Linewidth', 1.5);
    plot(z2.Time, z2.Data(:, 1), [Simulation.colors{1} '--'], 'Linewidth', 1.5);
    plot(z2.Time, z2.Data(:, 2), [Simulation.colors{2} '--'], 'Linewidth', 1.5);
    
    % Annotate
%     a_index = floor(1.0 / Simulation.Ts);
%     %x_pos = [z1.Time(a_index) + 3, z1.Time(a_index)];
%     y_pos1 = [z1.Data(end, 1) + 0.5, z1.Data(end, 1)] / 4 +0.54;
%     y_pos2 = [z1.Data(end, 2) - 0.5, z1.Data(end, 2)] / 4 +0.54;
% 
%     annotation('textarrow', [0.6 0.5], y_pos1,...
%         'String', 'x-coordinates');
%     annotation('textarrow', [0.5 0.6], y_pos2,...
%         'String', 'y-coordinates');
    % Axis stuff
    xlabel('time (s)'); ylabel('Amplitude (m)'); grid on;
    set(gca, 'FontSize', 18);
    
    saveMyFigure(gcf, [Simulation.name '_z'], 25, 10);
    if(Simulation.SavePNG)
        saveMyFigure(gcf, ['png/' Simulation.name '_z'], 25, 10, '.png');
    end


    %% Coordinate plot
    figure;
    % Plot system 1, x and y
    ref_value = -pi/4;
    ref_list = ones(numel(q.Time), 1)*ref_value;
    grid on;
    hold on;
    plot(q.Time, q.Data(:, 1), [Simulation.colors{1} '-'], 'Linewidth', 1.5);
    plot(q.Time, q.Data(:, 2), [Simulation.colors{2} '-'], 'Linewidth', 1.5);
    plot(q.Time, q.Data(:, 3), [Simulation.colors{3} '-'], 'Linewidth', 1.5);
    plot(q.Time, ref_list, ['k--'], 'Linewidth', 1.5);
    xlabel('time (s)'); ylabel('Angle (rad)'); grid on;
    set(gca, 'FontSize', 18);
    ylim([-pi/2, pi]);
    saveMyFigure(gcf, [Simulation.name '_q'], 25, 10);
    if(Simulation.SavePNG)
        saveMyFigure(gcf, ['png/' Simulation.name '_q'], 25, 10, '.png');
    end
    
    %% Storage function figure
%     figure;
%     plot(S.Time, S.Data, 'Linewidth', 1.5);
%     title('Storage function S(t)');
%     xlabel('time (s)'); ylabel('Amplitude'); grid on;
%     legend(system_legend)
%     saveMyFigure(gcf, [Simulation.name '_S'], 20, 10)
%     if(Simulation.SavePNG)
%         saveMyFigure(gcf, [Simulation.name '_S'], 20, 10, '.png');
%     end
end