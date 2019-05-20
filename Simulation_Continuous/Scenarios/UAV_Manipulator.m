Simulation.N = 2;
Simulation.l = 2;

% Only for manipulators: Set the mounting point
location{1} = [0.5; 0; 0];

%% Two Manipulators
[System{1}, SInfo{1}] = Manipulator_System(lambda, epsilon, location{1}, 1);
[System{2}, SInfo{2}] = UAV_System(lambda, epsilon, 2);

% Initial conditions
q01 = zeros(SInfo{1}.n, 1); p01 = zeros(SInfo{1}.n, 1);
q02 = zeros(SInfo{2}.n, 1); p02 = zeros(SInfo{2}.n, 1);
q02 = [0.5; 1.5; pi/2+pi/4];
%q01 = [-pi;0.2;0.4];
q01 = [0;0;0];