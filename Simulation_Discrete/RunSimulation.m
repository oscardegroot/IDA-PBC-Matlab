fprintf('--------------------------\n');
fprintf('Starting Simulation\n');
fprintf(['    Model: ' model '\n']);
tic;
load_system(model)

fprintf('Simulating...\n');
sim(model)

T_Sim = toc;
fprintf('Finished Simulation in %.0f seconds\n', toc);
