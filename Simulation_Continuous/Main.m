%% Main File for distributed IDA-PBC based protocol in simulink %%
% Created by: Oscar de Groot
clear all; clc; close all;

%% Main definitions
% Define the name of this simulation
Simulation.name = 'Presentation_pendulum_pbc';

% Choose the scenario to run
scenario = @Pendulum;

% Define relevant paths
system_path = 'Systems/';    % Path to system definitions
scenario_path = 'Scenarios/';      % Path to scenarios

% Add them to the matlab path
addpath(genpath(system_path));
addpath(genpath(scenario_path));

%% Load parameters
Parameters;

%% Simulation
fprintf('--------------------------\n');
fprintf('Simulating...\n');
tic;
load_system(model)
sim(model)
T_Sim = toc;

fprintf('Finished Simulation in %.0f seconds\n', toc);

GeneratePlots;
Animate(q, Simulation, t_out);
