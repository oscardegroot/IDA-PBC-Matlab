%% Set Parameters %%
% The scenario is defined in the main file

%% Simulation Settings
fprintf('Setting Parameters\n');
Simulation.Formation = false;
Simulation.duration = 20;
Simulation.dt = 0.05;

% Output Settings
Simulation.plots = true;
Simulation.SavePNG = true;
Simulation.GIF = true;

% Animation Settings
Simulation.life_animation = true;
Simulation.time_rate = 1;
Simulation.window_size = 2;
Simulation.colors = {'k', 'r', 'g'};

%% Define Gains
% r-passivity
lambda = 14;  %6         % Multiplier of z in the output (4)

% Network
gain = 1;               % Gain of the network (1)
Kd = gain*diag([1; 1]); % Network gain as matrix (eye) 
B = sqrt(gain)*eye(2);  % ST Line Impedance      (sqrt(gain)*eye)

%% Calculate the network gain
fprintf('1] Calculating Scattering Gain\n');
SetScatteringGain;

%% Load a Setup
fprintf('2] Loading scenario\n');
scenario();

%% Set initial simulation parameters
Simulation.systems = SInfo;

%% Define Delays
T = 0.05;
T12 = T;T21 = 2*T;
%T21 = T*5;

T23 = T*3;
T32 = T*2;

%% Define a formation if necessary
if(Simulation.Formation)
    Simulation.Formation_Goal = PointsToFormation(points);
else
    Simulation.Formation_Goal = zeros(Simulation.N*Simulation.l, Simulation.N);
end

Simulation.R = Simulation.Formation_Goal*System{1}.lambda/2;

%% Simulation Parameters
t_out = 0:Simulation.dt:Simulation.duration;
set_param(model, 'StopTime', num2str(Simulation.duration))

fprintf('Parameters set!\n\n');