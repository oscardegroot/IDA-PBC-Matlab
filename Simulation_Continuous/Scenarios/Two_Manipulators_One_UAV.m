%% Simulation with two manipulators and a UAV in Formation
Simulation.N = 3;
Simulation.l = 2;
model = 'Model_3Systems';

Simulation.Formation = true;

%% Two Manipulators
% Parameters{1}.location = [1; -1.5; 0];
% Parameters{2}.location = [-1; 1.5; 0];
Parameters{1}.location = [0.5; 0; 0];
Parameters{2}.location = [-0.5; 0; 0];

Parameters{1}.dVs = @(q) [0;0;0];Parameters{2}.dVs = @(q) [0;0;0];
for i = 1 : 2%0.5
    [System{i}, SInfo{i}] = Manipulator_System(lambda, 0.4, Parameters{i}, i);
end
q01 = zeros(SInfo{1}.n, 1); p01 = zeros(SInfo{1}.n, 1);
q02 = zeros(SInfo{2}.n, 1); p02 = zeros(SInfo{2}.n, 1);

%% One UAV
[System{3}, SInfo{3}] = UAV_System(lambda, 3);
q03 = zeros(SInfo{3}.n, 1); p03 = zeros(SInfo{3}.n, 1);

%% Initial conditions (Comment for zero)
q01 = [pi/2; 0.2; -0.3];
q02 = [-pi/4; -0.2; 0.3];
q03 = [0.5; 1.6; pi/2];
% q01 = [pi; -0.2; -0.3];
% q02= [-pi/4; 0.4; 0.3];
% q03 = [-1.5; 1.6; -pi/3];

%% Formation specified by points
point_1 = [SInfo{3}.l; 0];
point_2 = [-SInfo{3}.l; 0];
point_3 = [0; 0];
points = [point_1 point_2 point_3];