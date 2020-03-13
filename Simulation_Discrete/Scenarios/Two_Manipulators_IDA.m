%% Scenario for two manipulators reaching consensus %%
fprintf('2] Loading scenario with two manipulators for IDA-PBC control\n');

create_systems = true;

Simulation.N = 2;
Simulation.l = 2;
model = 'Model_2Systems_IDA';

Parameters{1}.location = [0.5; -1; 0];
Parameters{2}.location = [-1.5; 0.5; 0];

%% Two Manipulators
% for i = 1 : Simulation.N
%     [System{i}, SInfo{i}] = Manipulator_System(lambda, 0.3, Parameters{i}, i);
% end
if(create_systems)
    for i = 1 : Simulation.N
        [System{i}, SInfo{i}] = Manipulator_System(lambda, 0.4, Parameters{i}, i);
    end
else
    for i = 1 : Simulation.N
        S = load(['System' num2str(i)]);
        System{i} = S.System;
        SInfo{i} = S.SInfo;
    end
end

% Initial conditions
q01 = zeros(SInfo{1}.n, 1); p01 = zeros(SInfo{1}.n, 1);
q02 = zeros(SInfo{2}.n, 1); p02 = zeros(SInfo{2}.n, 1);
q01 = [pi/2; -0.1; 0.1];
q02 = [-0.1;-0.2;-0.3];

%% Discrete Parameters
% Define delays
if(Simulation.use_given_network == 0)
    delays_12 = getRandomDelays(L, Ts, 0.2, 0.3, 1.0);
    delays_21 = getRandomDelays(L, Ts, 0.6, 0.5, 1.0);
end


% Define dropouts
dropouts_12 = getRandomDropouts(L, 0.05);
dropouts_21 = getRandomDropouts(L, 0.1);
