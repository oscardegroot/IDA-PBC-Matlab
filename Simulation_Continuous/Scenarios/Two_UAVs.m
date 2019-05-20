Simulation.N = 2;
Simulation.l = 2;

%% Two Manipulators
for i = 1 : Simulation.N
    [System{i}, SInfo{i}] = UAV_System(lambda, epsilon, i);
end

% Initial conditions
q01 = zeros(SInfo{1}.n, 1); p01 = zeros(SInfo{1}.n, 1);
q02 = zeros(SInfo{2}.n, 1); p02 = zeros(SInfo{2}.n, 1);
q01 = [0.5; 0.5; pi/2];
q02 = [-1.5;-0.8;-pi/2];