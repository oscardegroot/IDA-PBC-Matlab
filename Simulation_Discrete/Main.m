%% Main File for distributed IDA-PBC based protocol in simulink %%
% Created by: Oscar de Groot
clear all; clc; close all;
profile on
%% Main definitions
% Define the name of this simulation
Simulation.name = 'automatica_ida_vs_delay_2';

% Choose the scenario to run
scenario = @Two_Manipulators_IDA;

% Define relevant paths
local_system_path = 'Systems/';
system_path = '../Systems/';    % Path to system definitions
scenario_path = 'Scenarios/';      % Path to scenarios

% Add them to the matlab path
addpath(genpath(local_system_path));
addpath(genpath(system_path));
addpath(genpath(scenario_path));

%% Load parameters
Parameters;

%% Simulation
sim_model = model;
RunSimulation;

if(Simulation.Comparison)
    Sim1.q = q; Sim1.IW = Input_Waves; Sim1.OW = Output_Waves; Sim1.S = S;
    Sim1.z1 = z1; Sim1.z2 = z2;
    sim_model = comparison_model;
    RunSimulation;
    Sim2.q = q; Sim2.IW = Input_Waves; Sim2.OW = Output_Waves; Sim2.S = S;
    Sim2.z1 = z1; Sim2.z2 = z2;
end

% If this is a comparison, compare the responses
if(Simulation.Comparison)
    ComparePlots;
% Otherwise generate an animation and plots
else
    GeneratePlots;
    if(Simulation.ShowWaves)
        AnimateWithWaves(q, Input_Waves, Output_Waves, Simulation);
    else
        Animate(q, Simulation, t_out);
    end
end
%profile viewer
