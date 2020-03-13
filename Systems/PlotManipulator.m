function PlotManipulator(y, S, SInfo, varargin)
    L = [0.5; 0.5; 0.5];
    
    p = SInfo.location(1:2);
    gamma = SInfo.location(3);
    

    
    for i = 1 : numel(y)
        p_prev = p;
        gamma = gamma + y(i);
        p = p + [L(i)*sin(gamma); L(i)*cos(gamma)];
        if(numel(varargin) > 0)
            plot([p_prev(1) p(1)], [p_prev(2) p(2)], 'LineStyle', varargin{2}, 'Color', varargin{1}, 'LineWidth', 1.5);
            plot(p(1), p(2), [S '*'], 'LineWidth', 1.5);
        else
            plot([p_prev(1) p(1)], [p_prev(2) p(2)], ['k-'], 'LineWidth', 1.5);
            plot(p(1), p(2), [S '*'], 'LineWidth', 1.5);
        end
    end
    

end