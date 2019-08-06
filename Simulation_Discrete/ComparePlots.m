if(Simulation.plots)
    figure;
    title('Network Comparison');
    subplot(211);
    plot(Sim1.z1.Time, Sim1.z1.Data, Sim1.z2.Time, Sim1.z2.Data, 'Linewidth', 1.5);
    title('WVM');
    xlabel('time (s)'); ylabel('Amplitude (m)'); grid on;
    %legend(system_legend)
    subplot(212);
    plot(Sim2.z1.Time, Sim2.z1.Data, Sim2.z2.Time, Sim2.z2.Data, 'Linewidth', 1.5);
    title('No WVM');
    xlabel('time (s)'); ylabel('Amplitude (m)'); grid on;
    %legend(system_legend)
    saveMyFigure(gcf, [Simulation.name '_z'], 20, 10);
    if(Simulation.SavePNG)
        saveMyFigure(gcf, [Simulation.name '_z'], 20, 10, '.png');
    end
    
    if(Simulation.ShowWaves)
        AnimateWithWaves(Sim1.q, Sim1.IW, Sim1.OW, Simulation, t_out);
        AnimateWithWaves(Sim2.q, Sim2.IW, Sim2.OW, Simulation, t_out);
    else
        Animate(Sim1.q, Simulation, t_out);
        Animate(Sim2.q, Simulation, t_out);
    end
end