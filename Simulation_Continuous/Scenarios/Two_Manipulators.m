%% Scenario for two manipulators reaching consensus %%
fprintf('2] Loading scenario with two manipulators\n');

Simulation.N = 2;
Simulation.l = 2;
model = 'Model_2Systems';

location{1} = [0.5; 0; 0];
location{2} = [-1.5; 0.5; 0];

%% Two Manipulators
for i = 1 : Simulation.N
    [System{i}, SInfo{i}] = Manipulator_System(lambda, 0.5, location{i}, i);
end

% Initial conditions
q01 = zeros(SInfo{1}.n, 1); p01 = zeros(SInfo{1}.n, 1);
q02 = zeros(SInfo{2}.n, 1); p02 = zeros(SInfo{2}.n, 1);
q01 = [0.1; 0.2; 0.3];
q02 = [-0.1;-0.2;-0.3];
