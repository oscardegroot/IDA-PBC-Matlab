%% Generate plots for a simulation setup %%
SetLegend;

%% Plot
if(Simulation.plots)
    %% Cooperative coordinate plot
    figure;
    title('Consensus in 2D');
    subplot(211);
    plot(z1.Time, z1.Data, 'Linewidth', 1.5);
    title('x');
    xlabel('time (s)'); ylabel('Amplitude (m)'); grid on;
    legend(system_legend)
    subplot(212);
    plot(z2.Time, z2.Data, 'Linewidth', 1.5);
    title('y');
    xlabel('time (s)'); ylabel('Amplitude (m)'); grid on;
    legend(system_legend)
    saveMyFigure(gcf, [Simulation.name '_z'], 20, 10);
    if(Simulation.SavePNG)
        saveMyFigure(gcf, [Simulation.name '_z'], 20, 10, '.png');
    end


    %% Storage function figure
    figure;
    plot(S.Time, S.Data, 'Linewidth', 1.5);
    title('Storage function S(t)');
    xlabel('time (s)'); ylabel('Amplitude'); grid on;
    legend(system_legend)
    saveMyFigure(gcf, [Simulation.name '_S'], 20, 10)
    if(Simulation.SavePNG)
        saveMyFigure(gcf, [Simulation.name '_S'], 20, 10, '.png');
    end
end