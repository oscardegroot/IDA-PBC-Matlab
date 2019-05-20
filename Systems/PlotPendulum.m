function PlotPendulum(y, S, SInfo)
    
    for i = 1 : numel(y)
        plot([0 SInfo.l*sin(y(1))], [0 SInfo.l*cos(y(1))], 'LineWidth', 1.5);
        plot(SInfo.l*sin(y(1)), SInfo.l*cos(y(1)), [S '*'], 'LineWidth', 1.5);
    end
    

end