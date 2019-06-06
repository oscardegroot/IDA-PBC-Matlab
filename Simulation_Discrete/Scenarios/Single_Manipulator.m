%% Scenario for two manipulators reaching consensus %%
fprintf('2] Loading scenario with a single manipulator\n');
Simulation.N = 1;
Simulation.l = 2;
model = 'SingleAgent_Manipulator';

Parameters{1}.location = [1.5; 0; 0];

%% Two Manipulators
for i = 1 : Simulation.N
    [System{i}, SInfo{i}] = Manipulator_System(lambda, 0.3, Parameters{i}, i);
end

% Initial conditions
q01 = zeros(SInfo{1}.n, 1); p01 = zeros(SInfo{1}.n, 1);
q01 = [pi/2; -0.1; 0.1];


