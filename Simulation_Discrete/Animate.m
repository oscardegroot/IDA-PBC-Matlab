%% Animate simulated systems %%
% Use the option Simulation.GIFs = true to generate a GIF of the animation
function Animate(qdata, Simulation, t_out)
    Nsteps = size(qdata.Data, 1);
    
    filename = ['GIFs/' Simulation.name '.gif'];
    
    % Time is slowed down by this rate
    time_rate = Simulation.time_rate;
    
    %% Format data
    y_all = {};
    col_start = 1;
    for i = 1 : Simulation.N
        col_end = col_start + Simulation.systems{i}.n;
        y_all{i} = qdata.Data(:, col_start:col_end-1);
        col_start = col_end;
    end
    
    % If Ts is smaller, generated data is slower than the simulation speed
    if(Simulation.dt < Simulation.Ts)
        t_out = 0:Simulation.Ts:Simulation.duration;
    end
    
    Tstep = t_out(2) - t_out(1);
    Tstep = Tstep / time_rate;
    t = t_out;
    
    %% Draw the animation
    h = figure;
   % waitforbuttonpress; pause(1);
    if(Simulation.life_animation)
        % Walk throught the simulation
        for i = 1 : Nsteps
            tic; clf; 

            %% Animate the trajectories
            hold on;grid on;axis square;
            for j = 1 : Simulation.N
                y = y_all{j};
                Simulation.systems{j}.plotf(y(i, :), Simulation.colors{j}, Simulation.systems{j});
            end
            title(['T = ' num2str(floor(t(i)))]);
            wsize = Simulation.window_size;
            woffset = [0; 0];
            xlim([woffset(1)-wsize, wsize+ woffset(1)]);
            ylim([woffset(2)-wsize, wsize+ woffset(2)]);

            % Save as GIF
            if(Simulation.GIF)
                if(mod(i, 5) == 0)
                    SaveFrameToGIF(filename, h, min(Simulation.dt, Simulation.Ts)/5, i);
                end
            end
            
            l = toc;
            pause(max(Tstep-l, 0));
        end
        hold on;
    end
   
if(Simulation.enhancement)
    % Find the middle location between the final points
    p_final = (feval(['a' num2str(1)], y_all{1}(end, :)') + ...
        feval(['a' num2str(2)], y_all{2}(end, :)'))./ 2;

end

%% Trajectories
hold on; axis square
    wsize = Simulation.window_size;
    woffset = [0; 0];
    xlim([woffset(1)-wsize, wsize+ woffset(1)]);
    ylim([woffset(2)-wsize, wsize+ woffset(2)]);
    for j = 1 : Simulation.N
        y = y_all{j};
        Simulation.systems{j}.plotf(y(1, :), Simulation.colors{j}, Simulation.systems{j},[0,0,0]+0.7, '-');
        hold on;
    end
    for j = 1 : Simulation.N
        y = y_all{j};
        
        Simulation.systems{j}.plotf(y(end, :), Simulation.colors{j}, Simulation.systems{j});
        hold on;
    end
    for j = 1 : Simulation.N
            y = y_all{j};
            hold on;
            if(strcmp(Simulation.systems{j}.name, 'Manipulator'))
                PlotManipulatorTrajectory(y, Simulation, j);
            elseif(strcmp(Simulation.systems{j}.name, 'Pendulum'))
                PlotManipulatorTrajectory(y, Simulation, j);
            else
                plot(y(:, 1), y(:, 2), [Simulation.colors{j} '--'], 'LineWidth', 1.5);
            end
    end
    
    %title('Trajectories in 2D');
    xlabel('x (m)'); ylabel('y (m)'); grid on;
    
    set(gca, 'FontSize', 18);
    set(gca, 'LineWidth', 2);
    
    if(Simulation.arrows)
        
        N_arrows = 2;
        
        % For each system
        for j = 1 : Simulation.N
            y = y_all{j};
            %position_tuner = 8;%20;
%             arrow_offset = 0.025;%0.015;
%              if(j == 1)
%                  arrow_offset = -arrow_offset;
%              end
            
            % Find out how large the trajectory is and divide it in N
            for i = 1 : size(y, 1)
                % Abs such that > always works?
                z(i, :) = feval(['a' num2str(j)], y(i, :)');
            end
             
            trajectory_length = norm(z(end, :) - z(1, :));
            
            % Draw N arrows
            for i = 1:N_arrows
                
                cur_length = trajectory_length / (N_arrows + 1) * i;
                
                % Find the position at the fraction
                %arrow_index = floor(i / position_tuner * size(y, 1));    
                arrow_index = -1; %find(norm(z - z(1,:)) > cur_length, 1, 'first');
                for k = 2 : size(z, 1)
                    if(norm(z(k, :) - z(1, :)) > cur_length)
                        arrow_index = k;
                        break;
                    end
                end
                
                z_loc = feval(['a' num2str(j)], y(arrow_index, :)');
                z_loc_next = feval(['a' num2str(j)], y(arrow_index + 1, :)');
                z_diff = z_loc_next - z_loc;
                arrow_angle = atan2(z_diff(2), z_diff(1)) * 180 / pi;
%Before
                %                  text_h = text(z_loc(1) ,z_loc(2) ,'>', 'FontSize', 26, ...
%                     'HorizontalAlignment', 'center');
%                 set(text_h, 'Rotation', arrow_angle);
                R = [cosd(arrow_angle) -sind(arrow_angle);...
                    sind(arrow_angle) cosd(arrow_angle)];
                arrow_offset = [0;0.04];
                z_loc = z_loc + R*arrow_offset;
               
                text_h = text(z_loc(1) ,z_loc(2) ,'>', 'FontSize', 26, ...
                    'HorizontalAlignment', 'center');
                set(text_h, 'Rotation', arrow_angle);
            end
        end
    end
    Simulation.systems{j}.plotf(y(1, :), Simulation.colors{j}, Simulation.systems{j},[0,0,0]+0.7, '-');

    
    % create a new pair of axes inside current figure
    if(Simulation.enhancement)
        % Mark the area of enhancement
        rectangle('Position', [p_final(1) - Simulation.zoom_size/2, p_final(2) - Simulation.zoom_size/2, Simulation.zoom_size, Simulation.zoom_size]);
        
        axes('position',[.22 .22 .25 .25])
        %axes('position',[p(1)-Simulation.zoom_size/2, p(2)-Simulation.zoom_size/2, Simulation.zoom_size, Simulation.zoom_size])
        box on % put box around new pair of axes
        t_relevant = 2/Simulation.Ts;
        PlotManipulatorTrajectory(y_all{1}(t_relevant:end, :), Simulation, 1);
        hold on;
        PlotManipulatorTrajectory(y_all{2}(t_relevant:end, :), Simulation, 2);
        hold on;
        % copy pasta, useful?
        for j = 1 : Simulation.N
        y = y_all{j};
        Simulation.systems{j}.plotf(y(1, :), Simulation.colors{j}, Simulation.systems{j},[0,0,0]+0.7, '-');
        hold on;
        end
        for j = 1 : Simulation.N
            y = y_all{j};

            Simulation.systems{j}.plotf(y(end, :), Simulation.colors{j}, Simulation.systems{j});
            hold on;
        end
        xlim([p_final(1) - Simulation.zoom_size/2, p_final(1) + Simulation.zoom_size/2]);
        ylim([p_final(2) - Simulation.zoom_size/2, p_final(2) + Simulation.zoom_size/2]);
        axis square
        %set(gca, 'FontSize', 16);
        % Hide numbering
        set(gca, 'yTickLabel', []);
        set(gca, 'xTickLabel', []);
%         set(gca, 'LineWidth', 2);
    end
    

    saveMyFigure(gcf, [Simulation.name '_xy'], 20, 20);
    if(Simulation.SavePNG)
        saveMyFigure(gcf, ['png/' Simulation.name '_xy'], 20, 20, '.png');
    end
end