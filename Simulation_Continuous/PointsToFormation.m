function [R] = PointsToFormation(Points)

    num_points = size(Points, 2);
    l = size(Points, 1);
    R = zeros(l*num_points, num_points);
    for i = 1 : num_points
        for j = 1 : num_points
            if(i == j)
                r_ij = zeros(l, 1);
                r_ji = zeros(l, 1);
            else
                p_i = Points(:, i);
                p_j = Points(:, j);
                
                r_ij = p_i - p_j;
                r_ji = p_j - p_i;
                
            end
            R((j - 1)*l + 1:j*l, i) = r_ij;
            R((i - 1)*l + 1:i*l, j) = r_ji;
        end
    end


end