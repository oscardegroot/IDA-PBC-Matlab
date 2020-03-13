function plotDrive(y, S, SInfo)
    l = SInfo.l; h = SInfo.h;
    plot(y(1), y(2), [S '*'], 'LineWidth', 1.5);
    p = [y(1); y(2)];
    %p_l = p - l*[cos(y(3));sin(y(3))];
    %p_r = p + l*[cos(y(3));sin(y(3))];
    %p_t = p + h*[-sin(y(3));cos(y(3))];
    
    %plot([p_l(1) p_r(1)], [p_l(2) p_r(2)], 'k', 'Linewidth', 1.5);
    rectangle('Position',[p(1) - SInfo.l / 2 p(2)-SInfo.l/2 SInfo.l SInfo.l],'Curvature',[1,1], 'Linewidth', 1.5);
%     plot([p_l(1) p_t(1)], [p_l(2) p_t(2)], S, 'Linewidth', 1.5);
%     plot([p_r(1) p_t(1)], [p_r(2) p_t(2)], S, 'Linewidth', 1.5);
end