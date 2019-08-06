%% Animate simulated systems %%
% Use the option Simulation.GIFs = true to generate a GIF of the animation
function AnimateWithWaves(qdata, w_in, w_out, Simulation, t_out)
    
    dec_rate = 2;
    Nsteps = size(qdata.Data, 1)/dec_rate;
    filename = ['GIFs/' Simulation.name '_waves.gif'];
    
    % Time is slowed down by this rate
    time_rate = Simulation.time_rate;
    w_in = w_in.Data;
    w_out = w_out.Data;
    
    %% Format data
    y_all = {};
    col_start = 1;
    for i = 1 : Simulation.N
        col_end = col_start + Simulation.systems{i}.n;
        y_all{i} = qdata.Data(:, col_start:col_end-1);
        
        % Decimate
        y_temp = [];
        for j = 1 : size(y_all{i}, 2)
           y_temp = [y_temp decimate(y_all{i}(:, j), dec_rate)];
        end
        y_all{i} = y_temp;
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
    set(h, 'Position',  [100, 100, 1400, 600])
    waitforbuttonpress; pause(1);
    if(Simulation.life_animation)
        % Walk throught the simulation
        for i = 1 : Nsteps
            tic; clf; 
            subplot(2,2,[1,3]);
            %% Animate the trajectories
            hold on;grid on;axis square;
            for j = 1 : Simulation.N
                y = y_all{j};
                Simulation.systems{j}.plotf(y(i, :), Simulation.colors{j}, Simulation.systems{j});
            end
            title(['T = ' num2str(floor(t(i*dec_rate)))]);
            wsize = Simulation.window_size;
            woffset = [0; 0];
            xlim([woffset(1)-wsize, wsize+ woffset(1)]);
            ylim([woffset(2)-wsize, wsize+ woffset(2)]);
            

            subplot(222);
            hold on;grid on;
            title('Input Waves');
            xlim([0, t_out(end)]);
            ylim([min(min(w_in)), max(max(w_in))]);
            for j = 1 : 4
                hold on;
                stairs(t_out(1:i*dec_rate), w_in(1:i*dec_rate, j), 'LineWidth', 1);
            end

            subplot(224);
            hold on;grid on;
            title('Output Waves');
            xlim([0, t_out(end)]);
            ylim([min(min(w_out)), max(max(w_out))]);
            for j = 1 : 4
                hold on;
                stairs(t_out(1:i*dec_rate), w_out(1:i*dec_rate, j), 'LineWidth', 1);
            end
            
            % Save as GIF
            if(Simulation.GIF)
                SaveFrameToGIF(filename, h, Tstep*dec_rate, i);
            end
            
            l = toc;
            pause(max(Tstep-l, 0));
        end
        hold on;
    end
   
%% Trajectories
hold on;
subplot(2,2,[1,3]);
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
    saveMyFigure(gcf, [Simulation.name '_xy'], 40, 20);
    if(Simulation.SavePNG)
        saveMyFigure(gcf, [Simulation.name '_xy'], 40, 20, '.png');
    end
end