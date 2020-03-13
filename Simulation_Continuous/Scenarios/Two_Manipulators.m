%% Scenario for two manipulators reaching consensus %%
fprintf('3] Loading scenario with two manipulators\n');

create_systems = false;

Simulation.N = 2;
Simulation.l = 2;
model = 'Model_2Systems';

%% Initial conditions
% q01 = zeros(SInfo{1}.n, 1); p01 = zeros(SInfo{1}.n, 1);
% q02 = zeros(SInfo{2}.n, 1); p02 = zeros(SInfo{2}.n, 1);
q01 = [0.2; -0.1; 0.1];
q02 = [-pi;-0.2;-0.3];
Parameters{1}.location = [1.5; 0.0; 0];
Parameters{2}.location = [-1; -0.0; 0];

%% Two Manipulators
if(create_systems)
    for i = 1 : Simulation.N
        [System{i}, SInfo{i}] = Manipulator_System(lambda, 0.3, Parameters{i}, i);
    end
else
    for i = 1 : Simulation.N
        S = load(['System' num2str(i)]);
        System{i} = S.System;
        SInfo{i} = S.SInfo;
    end
end

p01 = zeros(SInfo{1}.n, 1); p02 = zeros(SInfo{2}.n, 1);
