%% Parameter file
fprintf('Setting Parameters\n');
Simulation.Comparison = false;
Simulation.Formation = false;
Simulation.duration = 20;
Simulation.create_systems = false;

% Output Settings
Simulation.plots = true;
Simulation.enhancement = true;
Simulation.arrows = true;
Simulation.zoom_size = 0.5;%0.65;  % 0.25
Simulation.SavePNG = true;
Simulation.GIF = true;
Simulation.ShowWaves = false;
Simulation.SplitWaves = false;

% Delay settings
Simulation.delays = true;
Simulation.dropouts = true;
Simulation.delay_is_variable = true;
Simulation.use_given_network = true; % Note: duration needs to be equal

% Animation Settings
Simulation.life_animation = false;
Simulation.time_rate = 1;%3;
Simulation.window_size = 2;
Simulation.colors = {'b', 'r', 'g'};

%% Discrete time parameters
% Sampling time
Ts = 0.01; Simulation.Ts = Ts;
Simulation.dt = Ts/2;

% Default Constant Delays
T12 = ceil(0.1/Ts);
T21 = ceil(0.5/Ts);
T23 = ceil(0.3/Ts);
T32 = ceil(0.2/Ts);

% Simulation timesteps
L = Simulation.duration/Ts+1;
t_delays = Ts*(1:L);

if(Simulation.use_given_network)
    if(exist('delays.mat'))
        load('delays.mat');
    else
        delays_12 = getRandomDelays(L, Ts, 0.2, 0.3, 1.0);
        delays_21 = getRandomDelays(L, Ts, 0.6, 0.5, 1.0);
        delays_23 = getRandomDelays(L, Ts, 0.2, 0.3, 1.0);
        delays_32 = getRandomDelays(L, Ts, 0.6, 0.5, 1.0);
        delays_13 = getRandomDelays(L, Ts, 0.2, 0.3, 1.0);
        delays_31 = getRandomDelays(L, Ts, 0.6, 0.5, 1.0);
        save('delays.mat', 'delays_12', 'delays_21');
    end
end

%% Define Gains
lambda = 1;

gain = 6.0;%6.0; 4.0 voor beide!
Kd = gain*diag([1; 1]);       % Network gain
B = sqrt(gain)*eye(2);   % ST Line Impedance

%% Calculate the network gain
fprintf('1] Calculating Scattering Gain\n');
SetScatteringGain;

%% Load a Setup
scenario();

%% Set delays / dropouts constant is specified
% Constant delay parameters
if(Simulation.delays == 0)
    delays_12 = zeros(L, 1);
    delays_21 = zeros(L, 1);
elseif(Simulation.delay_is_variable == 0)
    delays_12 = T12*ones(L, 1);
    delays_21 = T21*ones(L, 1);
end

% No dropouts paramaters
if(Simulation.dropouts == 0)
    dropouts_12 = zeros(L, 1);
    dropouts_21 = zeros(L, 1);
end

start_delays(1, 1) = delays_21(1) * Ts;
start_delays(1, 2) = delays_12(1) * Ts;

%% Set initial simulation parameters
Simulation.systems = SInfo;

%% Define a formation if necessary
if(Simulation.Formation)
    Simulation.Formation_Goal = PointsToFormation(points);
else
    Simulation.Formation_Goal = zeros(Simulation.N*Simulation.l, Simulation.N);
end

Simulation.R = Simulation.Formation_Goal*lambda/2;

%% Simulation Parameters
% Time parameters
t_out = 0:Simulation.dt:Simulation.duration;


fprintf('Parameters set!\n');