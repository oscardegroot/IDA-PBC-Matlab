clear all; clc; close all;

% The simulink model this main document refers to
model = 'Model_2Systems';
sim_name = 'm2_rvalidation';
system_path = '../Systems/';

% Add the systems to the path
addpath(genpath(system_path));

% Load parameters
Parameters;

%% Simulation
fprintf('Simulating...\n');
tic;
load_system(model)
sim(model)
T_Sim = toc;

fprintf('Finished Simulation in %.1f seconds\n', toc);

%% Plot
if(Simulation.plots)
    figure;
    title('Consensus in 2D');
    subplot(211);
    plot(z1.Time, z1.Data, 'Linewidth', 1.5);
    title('x');
    xlabel('time (s)'); ylabel('Amplitude (m)'); grid on;
    legend([SInfo1.legend SInfo2.legend]);
    subplot(212);
    plot(z2.Time, z2.Data, 'Linewidth', 1.5);
    title('y');
    xlabel('time (s)'); ylabel('Amplitude (m)'); grid on;
    legend([SInfo1.legend SInfo2.legend]);
    saveMyFigure(gcf, [sim_name '_z'], 20, 10);

    figure;
    plot(S_1.Time, S_1.Data, 'Linewidth', 1.5);
    title('Storage function S(t)');
    xlabel('time (s)'); ylabel('Amplitude'); grid on;
    saveMyFigure(gcf, [sim_name '_S'], 20, 10)
end

Animate(q, Simulation, t_out);
title('Trajectories in 2D');
xlabel('x (m)'); ylabel('y (m)'); grid on;
saveMyFigure(gcf, [sim_name '_xy'], 20, 20);
