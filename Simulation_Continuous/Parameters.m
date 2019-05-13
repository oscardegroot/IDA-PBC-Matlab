%% Parameter file
% Simulation Settings
fprintf('Setting Parameters\n');
Simulation.Formation = false;
Simulation.duration = 10;
Simulation.dt = 0.05;

% Output Settings
Simulation.plots = true;

% Animation Settings
Simulation.life_animation = true;
Simulation.time_rate = 1;
Simulation.window_size = 2;
Simulation.colors = {'k', 'r', 'g'};

%% Define Gains
% r-passivity
lambda = 4;  %6           % Multiplier of z in the output (4)
epsilon = 5e-1;         % Levenberg constant    (5e-1)

% Network
gain = 1;               % Gain of the network (1)
Kd = gain*diag([1; 1]); % Network gain as matrix (eye) 
B = sqrt(gain)*eye(2);  % ST Line Impedance      (sqrt(gain)*eye)

% Damping
Kv = 0*eye(2);          % Dampens the response via z_dot  (0)

%% Calculate the network gain
fprintf('Loading Laplace\n');
LaplaceScattering;

%% Define systems
fprintf('Loading Systems\n');

%% Load a Setup
setup()

%% Set initial simulation parameters
% Fun fact: Only the systems here are drawn
Simulation.systems = SInfo;

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

Simulation.R = Simulation.Formation_Goal*lambda/2;

%% Validation of R
Vr0 = 0; % Could be more extensive (but needs calculation for each agent)

%% Simulation Parameters
% Time parameters
t_out = 0:Simulation.dt:Simulation.duration;
set_param(model, 'StopTime', num2str(Simulation.duration))

fprintf('Parameters set\n');