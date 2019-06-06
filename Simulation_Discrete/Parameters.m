%% Parameter file
fprintf('Setting Parameters\n');
Simulation.Comparison = false;
Simulation.Formation = false;
Simulation.duration = 10;


% Output Settings
Simulation.plots = true;
Simulation.SavePNG = true;
Simulation.GIF = true;
Simulation.ShowWaves = true;

% Delay settings
Simulation.delay_is_variable = true;
Simulation.dropouts = true;

% Animation Settings
Simulation.life_animation = true;
Simulation.time_rate = 1;
Simulation.window_size = 2;
Simulation.colors = {'k', 'r', 'g'};

%% Discrete time parameters
% Sampling time
Ts = 0.05; Simulation.Ts = Ts;
Simulation.dt = Ts/2;

% Default Constant Delays
T12 = ceil(0.1/Ts);
T21 = ceil(0.5/Ts);
T23 = ceil(0.3/Ts);
T32 = ceil(0.2/Ts);

% Simulation timesteps
L = Simulation.duration/Ts+1;
t_delays = Ts*(1:L);

% Constant delay parameters
if(Simulation.delay_is_variable == 0)
    delays_12 = T12*ones(L, 1);
    delays_21 = T21*ones(L, 1);
end

% No dropouts paramaters
if(Simulation.dropouts == 0)
    dropouts_12 = zeros(L, 1);
    dropouts_21 = dropouts_12;
end

%% Define Gains
lambda = 6;

gain = 1;
Kd = gain*diag([1; 1]);       % Network gain
B = sqrt(gain)*eye(2);   % ST Line Impedance

%% Calculate the network gain
fprintf('1] Calculating Scattering Gain\n');
SetScatteringGain;

%% Load a Setup
scenario();

%% Set initial simulation parameters
Simulation.systems = SInfo;

%% Define a formation if necessary
if(Simulation.Formation)
    Simulation.Formation_Goal = PointsToFormation(points);
else
    Simulation.Formation_Goal = zeros(Simulation.N*Simulation.l, Simulation.N);
end

Simulation.R = Simulation.Formation_Goal*System{1}.lambda/2;

%% Simulation Parameters
% Time parameters
t_out = 0:Simulation.dt:Simulation.duration;
set_param(model, 'StopTime', num2str(Simulation.duration))

fprintf('Parameters set!\n');