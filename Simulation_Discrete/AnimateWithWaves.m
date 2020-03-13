%% Animate simulated systems %%
% Use the option Simulation.GIFs = true to generate a GIF of the animation
function AnimateWithWaves(qdata, w_in, w_out, Simulation)
    
    dec_rate = Simulation.time_rate;
    Nsteps = size(qdata.Data, 1)/dec_rate;
    filename = ['GIFs/' Simulation.name '_waves.gif'];
    
    % Time is slowed down by this rate
    %time_rate = Simulation.time_rate/dec_rate;
    w_in_temp = [];
    w_out_temp = [];
    for i = 1 : size(w_in.Data, 2)
        w_in_temp = [w_in_temp w_in.Data(1:dec_rate:end, i)];
        w_out_temp = [w_out_temp w_out.Data(1:dec_rate:end, i)];
    end
    
    w_in = w_in_temp;
    w_out = w_out_temp;
    
    %% Format data
    y_all = {};
    col_start = 1;
    for i = 1 : Simulation.N
        col_end = col_start + Simulation.systems{i}.n;
        y_all{i} = qdata.Data(:, col_start:col_end-1);
        
        % Decimate
        y_temp = [];
        for j = 1 : size(y_all{i}, 2)
           y_temp = [y_temp y_all{i}(1:dec_rate:end, j)];
        end
        y_all{i} = y_temp;
        col_start = col_end;
    end
    
    % If Ts is smaller, generated data is slower than the simulation speed
%     if(Simulation.dt < Simulation.Ts)
%         t_out = 0:Simulation.Ts:Simulation.duration;
%     end
    t_out = decimate(qdata.Time, dec_rate);
    Tstep = t_out(2) - t_out(1);
%     Tstep = Tstep * time_rate;
%     t = t_out;
    
    %% Draw the animation
    h = figure;
    if(Simulation.SplitWaves)
        g = figure;
    end
    
    set(h, 'Position',  [100, 100, 1400, 600])
    set(gca, 'FontSize', 16);
    waitforbuttonpress; pause(0.2);
    if(Simulation.life_animation)
        % Walk throught the simulation
        for i = 1 : Nsteps
            tic; clf; 
            
            if(Simulation.SplitWaves == 0)
                subplot(2,2,[1,3]);
            else
                set(0, 'CurrentFigure', h);
                clf;
            end
            
            %% Animate the trajectories
            hold on;grid on;axis square;
            for j = 1 : Simulation.N
                y = y_all{j};
                Simulation.systems{j}.plotf(y(i, :), Simulation.colors{j}, Simulation.systems{j});
            end
            xlabel('x (m)');
            ylabel('y (m)');
            title(['T = ' num2str(floor(t_out(i)))]);
            wsize = Simulation.window_size;
            woffset = [0; 0];
            xlim([woffset(1)-wsize, wsize+ woffset(1)]);
            ylim([woffset(2)-wsize, wsize+ woffset(2)]);
            
            if(Simulation.SplitWaves == 0)
                subplot(222);
            else
                set(0, 'CurrentFigure', g);
                subplot(211);
            end
            
            hold on;grid on;
            title('Input Waves');
            xlim([0, t_out(end)]);
            ylim([min(min(w_in))*1.2, max(max(w_in))*1.2]);
            for j = 1 : 4
                hold on;
                if(j == 1)
                    stairs(t_out(1:i), w_in(1:i, j), 'b', 'LineWidth', 1.5);
                elseif(j == 2)
                    stairs(t_out(1:i), w_in(1:i, j), 'b--', 'LineWidth', 1.5);
                elseif(j==3)
                    stairs(t_out(1:i), w_in(1:i, j), 'r', 'LineWidth', 1.5);
                else
                    stairs(t_out(1:i), w_in(1:i, j), 'r--', 'LineWidth', 1.5);
                end
            end
            xlabel('Time (s)');
            ylabel('Amplitude');
            if(Simulation.SplitWaves == 0)
                subplot(224);
            else
                set(0, 'CurrentFigure', g);
                subplot(212);
            end
            hold on;grid on;
            title('Output Waves');
            xlim([0, t_out(end)]);
            ylim([min(min(w_out))*1.2, max(max(w_out))*1.2]);
            for j = 1 : 4
                hold on;
                if(j == 1)
                    stairs(t_out(1:i), w_out(1:i, j), 'b', 'LineWidth', 1.5);
                elseif(j == 2)
                    stairs(t_out(1:i), w_out(1:i, j), 'b--', 'LineWidth', 1.5);
                elseif(j==3)
                    stairs(t_out(1:i), w_out(1:i, j), 'r', 'LineWidth', 1.5);
                else
                    stairs(t_out(1:i), w_out(1:i, j), 'r--', 'LineWidth', 1.5);
                end
            end
            xlabel('Time (s)');
            ylabel('Amplitude');
            % Save as GIF
            if(Simulation.GIF)
                SaveFrameToGIF(filename, h, Tstep, i);
            end
            
            l = toc;
            pause(max(Tstep-l, 0));
        end
        hold on;
    end
   
%% Trajectories
hold on;
if(Simulation.SplitWaves)
    set(0, 'CurrentFigure', h);
else
    subplot(2,2,[1,3]);
end
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
    title('2D Trajectories');
    xlabel('x (m)'); ylabel('y (m)'); grid on;
    saveMyFigure(gcf, [Simulation.name '_xy'], 40, 20);
    if(Simulation.SavePNG)
        saveMyFigure(gcf, [Simulation.name '_xy'], 40, 20, '.png');
    end
    
    if(Simulation.SplitWaves)
        set(0, 'CurrentFigure', g);
        saveMyFigure(g, [Simulation.name '_waves'], 40, 20);
        if(Simulation.SavePNG)
            saveMyFigure(g, [Simulation.name '_waves'], 40, 20, '.png');
        end
    end
end