clear all; clc; close all;

% The simulink model this main document refers to
model = 'Model_2Systems';
sim_name = 'pseudo';
system_path = '../Systems/';

addpath(genpath(system_path));

% Load parameters
Parameters;

%% Simulation
fprintf('Starting simulation\n');
tic;
load_system(model)
sim(model)
T_Sim = toc;

fprintf('Finished Simulation in %.1f seconds\n', toc);

%% Plot
figure;
plot(q.Time, q.Data, 'Linewidth', 1.5);
title('Consensus in 2D');
xlabel('time (s)'); ylabel('Amplitude (m)'); grid on;
legend([SInfo1.legend SInfo2.legend]);
saveMyFigure(gcf, [sim_name '_x'], 20, 10)
Animate(q, Simulation, t_out);
title('Trajectories in 2D');
xlabel('x (m)'); ylabel('y (m)'); grid on;
saveMyFigure(gcf, [sim_name '_xy'], 20, 20)
