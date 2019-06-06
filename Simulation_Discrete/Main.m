%% Main File for distributed IDA-PBC based protocol in simulink %%
% Created by: Oscar de Groot
clear all; clc; close all;

%% Main definitions
% Define the name of this simulation
Simulation.name = 'discrete_2m_wvm';

% Choose the scenario to run
scenario = @Two_Manipulators;

% Define relevant paths
system_path = '../Systems/';    % Path to system definitions
scenario_path = 'Scenarios/';      % Path to scenarios

% Add them to the matlab path
addpath(genpath(system_path));
addpath(genpath(scenario_path));

%% Load parameters
Parameters;

%% Simulation
sim_model = model;
RunSimulation;

if(Simulation.Comparison)
    sim_model = comparison_model;
    RunSimulation;
end

% If this is a comparison, compare the responses
if(Simulation.Comparison)
    ComparePlots;
% Otherwise generate an animation and plots
else
    GeneratePlots;
    if(Simulation.ShowWaves)
        AnimateWithWaves(q, Input_Waves, Output_Waves, Simulation, t_out);
    else
        Animate(q, Simulation, t_out);
    end
end
