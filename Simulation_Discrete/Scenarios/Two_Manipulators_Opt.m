%% Scenario for two manipulators reaching consensus %%
fprintf('2] Loading scenario with two manipulators\n');

Simulation.N = 2;
Simulation.l = 2;
model = 'Model_2Systems_Opt';

Parameters{1}.location = [0.5; 0; 0];
Parameters{2}.location = [-0.5; 0; 0];

%% Two Manipulators
for i = 1 : Simulation.N
    [System{i}, SInfo{i}] = Manipulator_System(lambda, 0.3, Parameters{i}, i);
end

% Initial conditions
q01 = zeros(SInfo{1}.n, 1); p01 = zeros(SInfo{1}.n, 1);
q02 = zeros(SInfo{2}.n, 1); p02 = zeros(SInfo{2}.n, 1);
q01 = [pi/2; -0.1; 0.1];
q02 = [-0.1;-0.2;-0.3];

%% Discrete Parameters
% Define delays
% delays_12 = getRampDelays(L, Ts, 0.2, 0.3, 1.0);
% delays_21 = getRandomDelays(L, Ts, 0.6, 0.5, 1.0);
% 
% % Define dropouts
% dropouts_12 = getRandomDropouts(L, 0.05);
% dropouts_21 = getRandomDropouts(L, 0.1);
