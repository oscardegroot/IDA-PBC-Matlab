%% Parameter file
% Simulation Settings
fprintf('Setting Parameters\n');
Simulation.Formation = false;
Simulation.duration = 10;
Simulation.dt = 0.05;

% Animation Settings
Simulation.life_animation = true;
Simulation.time_rate = 1;
Simulation.window_size = 2.0;
Simulation.colors = {'k', 'r', 'g'};

%% Define Gains
% r-passivity
lambda = 6;             % Multiplier of z in the output (4)
epsilon = 5e-1;         % Levenberg constant    (5e-1)

% Network
gain = 2;               % Gain of the network (1)
Kd = gain*diag([1; 1]); % Network gain as matrix (eye) 
B = sqrt(gain)*eye(2);  % ST Line Impedance      (sqrt(gain)*eye)

% Damping
Kv = 2*eye(2);          % Dampens the response via z_dot  (0)

%% Calculate the network gain
fprintf('Loading Laplace\n');
LaplaceScattering;

%% Define systems
fprintf('Loading Systems\n');

% Only for manipulators: Set the mounting point
location_1 = [0; 0; 0];
location_2 = [-1; -0.5; 0];
location_3 = [-1.5; -0.25; 0];
[System1, SInfo1] = Manipulator_System(lambda, epsilon, location_1, 1);%Manipulator_System(lambda, epsilon,location_1, 1);
[System2, SInfo2] = Manipulator_System(lambda, epsilon, location_2, 2);
%[System3, SInfo3] = UAV_System(lambda, epsilon, 3);

% Initial conditions
q01 = zeros(SInfo1.n, 1); p01 = zeros(SInfo1.n, 1);
q02 = zeros(SInfo2.n, 1); p02 = zeros(SInfo2.n, 1);
% q03 = zeros(SInfo3.n, 1); p03 = zeros(SInfo3.n, 1);
q01 = [0;0;0];
q02 = [pi;0;0];

%% Set initial simulation parameters
% Fun fact: Only the systems here are drawn
Simulation.systems = {SInfo1; SInfo2};%; SInfo3}; 
Simulation.N = numel(Simulation.systems);
Simulation.l = 2;

%% Define Delays
T = 0.1;
T12 = T;
T21 = T*5;
T23 = T*3;
T32 = T*2;

%% Define a formation if necessary
if(Simulation.Formation)
    point_1 = [SInfo3.l; 0];
    point_2 = [-SInfo3.l; 0];
    point_3 = [0; 0];
    points = [point_1 point_2 point_3];
    Simulation.Formation_Goal = PointsToFormation(points);
    
else
    Simulation.Formation_Goal = zeros(Simulation.N*Simulation.l, Simulation.N);
end

Simulation.R = Simulation.Formation_Goal*System1.lambda/2;

%% Validation of R
Vr0 = 0; % Could be more extensive (but needs calculation for each agent)

%% Simulation Parameters
% Time parameters
t_out = 0:Simulation.dt:Simulation.duration;
set_param(model, 'StopTime', num2str(Simulation.duration))

fprintf('Parameters set\n');