%% Animate simulated systems
function Animate(qdata, Simulation, t_out)
    Nsteps = size(qdata.Data, 1);
    
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
    figure;
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
            else
                plot(y(:, 1), y(:, 2), [Simulation.colors{j} '--'], 'LineWidth', 1.5);
            end
    end
end