function PlotManipulatorTrajectory(y, Simulation, index)
    
    SInfo = Simulation.systems{index};
    load(SInfo.filename);
    
    for i = 1 : size(y, 1)
        z(i, :) = System.a(y(i, :)');
    end
    
    plot(z(:, 1), z(:, 2), [Simulation.colors{index} '--'], 'LineWidth', 1.5);
    
    % Formation...
%     z_e = [z(end, 1); z(end, 2)];
%     if(index == 1)
%         z_f = z_e + Simulation.Formation_Goal';
%     else
%         z_f = z_e - Simulation.Formation_Goal';
%     end
%     plot([z_e(1); z_f(1)], [z_e(2); z_f(2)], [Simulation.colors{index} 'o--']);

end