fprintf('--------------------------\n');
fprintf('Starting Simulation\n');
fprintf(['    Model: ' sim_model '\n']);

set_param(sim_model, 'StopTime', num2str(Simulation.duration))

tic;
load_system(sim_model)

fprintf('Simulating...\n');
sim(sim_model)

T_Sim = toc;
fprintf('Finished Simulation in %.0f seconds\n', toc);