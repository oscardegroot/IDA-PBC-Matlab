Simulation.N = 1;
Simulation.l = 1;
model = 'SingleAgent_Pendulum';
Simulation.plots = false;
Simulation.duration = 10;

%% Two Manipulators
[System{1}, SInfo{1}] = Pendulum_System(lambda, 1);

% Initial conditions
q01 = pi/2; p01 = 0;