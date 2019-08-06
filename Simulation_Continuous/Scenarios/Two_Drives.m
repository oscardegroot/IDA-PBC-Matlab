%% Scenario for two manipulators reaching consensus %%
fprintf('2] Loading scenario with two manipulators\n');

Simulation.N = 2;
Simulation.l = 2;
model = 'Model_2Systems';

Parameters{1}.location = [0.5; 0; 0];
Parameters{2}.location = [-1.5; 0.5; 0];

%% Two Diff Drives
for i = 1 : Simulation.N
    [System{i}, SInfo{i}] = Drive_System(lambda, i);
end

% Initial conditions
q01 = zeros(SInfo{1}.n, 1); p01 = zeros(SInfo{1}.n, 1);
q02 = zeros(SInfo{2}.n, 1); p02 = zeros(SInfo{2}.n, 1);
q01 = [1.0; 0.5; 0.2];
q02 = [-0.8;-0.3;-0.3];
