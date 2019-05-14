Simulation.N = 2;
Simulation.l = 2;
model = 'Model_2Systems';

location{1} = [0.5; 0; 0];
location{2} = [-0.5; 0; 0];

%% Two Manipulators
for i = 1 : Simulation.N
    [System{i}, SInfo{i}] = Manipulator_System(lambda, epsilon, location{i}, i);
end

% Initial conditions
q01 = zeros(SInfo{1}.n, 1); p01 = zeros(SInfo{1}.n, 1);
q02 = zeros(SInfo{2}.n, 1); p02 = zeros(SInfo{2}.n, 1);
q01 = [0.1; 0.2; 0.3];
q02 = [-0.1;-0.2;-0.3];


% location{1} = [0.5; 0; 0];
% location{2} = [-0.5; 0; 0];
% [System{1}, SInfo{1}] = Manipulator_System(lambda, epsilon, location{1}, 1);
% [System{2}, SInfo{2}] = Manipulator_System(lambda, epsilon, location{2}, 2);
% 
% % Initial conditions
% q01 = zeros(SInfo{1}.n, 1); p01 = zeros(SInfo{1}.n, 1);
% q02 = zeros(SInfo{2}.n, 1); p02 = zeros(SInfo{2}.n, 1);
% 
% %% Some intials
% % q01 = [pi/2; 0; 0];
% % q02 = [-pi;0;0];
% q01 = [0.3; 0.4; 0.2];
% q02 = [-0.5; -0.4; -0.3];
%q02 = [-pi/2;0.1;0.4];
% q01 = [0.1; 0.2; 0.3];
% q02 = [-0.1;-0.2;-0.3];
%q02 = [-0.5; -1; 0.9];
%q01 = [0.4;-1;-1];




