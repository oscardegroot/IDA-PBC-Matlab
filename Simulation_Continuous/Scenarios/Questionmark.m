%% Simulation with two manipulators and a UAV in Formation
fprintf('2] Loading scenario to form a question mark\n');
Simulation.N = 3;
Simulation.l = 2;
model = 'Model_3Systems';

Simulation.Formation = true;
Simulation.duration = 30;

%% Two Manipulators
Parameters{2}.location = [0.5*(1+0.5*sqrt(2)); -0.5*(2+0.5*sqrt(2)); 0];
Parameters{1}.dVs = @(q) [0;10*(q(2) - pi/4);0];
Parameters{2}.dVs = @(q) [0;10*(q(2) - pi/4);10*(q(3) + pi/4)];

for i = 1 : 2
    [System{i}, SInfo{i}] = Manipulator_System(lambda, 0.5, Parameters{i}, i);
end
q01 = zeros(SInfo{1}.n, 1); p01 = zeros(SInfo{1}.n, 1);
q02 = zeros(SInfo{2}.n, 1); p02 = zeros(SInfo{2}.n, 1);

target_z1 = [0.5*(1+sqrt(2)); 0];
[System{1}, SInfo{1}] = MakeLeader(System{1}, SInfo{1}, target_z1, eye(2));

%% One UAV
[System{3}, SInfo{3}] = UAV_System(lambda, 3);
q03 = zeros(SInfo{3}.n, 1); p03 = zeros(SInfo{3}.n, 1);

%% Initial conditions (Comment for zero)
q01 = [0.1; 0.2; 0.3];
q02= [-0.3; -0.2; -0.3];
q03 = [-1.5; -1; -pi/3];

%% Formation specified by points
point_1 = [0; 0];
point_2 = [0; 0];
point_3 = [-0.5*0.5*sqrt(2); -0.3 - 0.5*(2+0.5*sqrt(2))];
points = [point_1 point_2 point_3];