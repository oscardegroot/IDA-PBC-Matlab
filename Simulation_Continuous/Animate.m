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
        
    Tstep = t_out(2) - t_out(1);
    Tstep = Tstep / time_rate;
    t = t_out;
    
    %% Draw the animation
    h = figure;
    waitforbuttonpress; pause(1);
    if(Simulation.life_animation)
    
        for i = 1 : Nsteps
            tic; clf;hold on; grid on;axis square;
            for j = 1 : Simulation.N
                y = y_all{j};
                Simulation.systems{j}.plotf(y(i, :), Simulation.colors{j}, Simulation.systems{j});
            end
            title(['T = ' num2str(floor(t(i)))]);
            wsize = Simulation.window_size;
            woffset = [0; 0];
            xlim([woffset(1)-wsize, wsize+ woffset(1)]);
            ylim([woffset(2)-wsize, wsize+ woffset(2)]);
            
            if(Simulation.GIF)
                SaveFrameToGIF(filename, h, Simulation.dt, i);
            end
            
            l = toc;
            pause(max(Tstep-l, 0));
        end
        hold on;
    end
   
%% Trajectories
hold on;
    for j = 1 : Simulation.N
        y = y_all{j};
        Simulation.systems{j}.plotf(y(1, :), Simulation.colors{j}, Simulation.systems{j});
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
    title('Trajectories in 2D');
    xlabel('x (m)'); ylabel('y (m)'); grid on;
    saveMyFigure(gcf, [Simulation.name '_xy'], 20, 20);
end