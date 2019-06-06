%% Assume the only relevant variable is the last coordinate and its derivative
% I could also fit a linear function on each entry of the matrices?
% Possibly not due to cosines etc.

filename = '../Simulation_Continuous/Systems/UAV_n1';
%% Parameters
limits_q = [-2*pi; 2*pi]; resolution_q = 1e-1; 
N_q = ceil(abs(limits_q(2)-limits_q(1))/resolution_q);
q_range = linspace(limits_q(1), limits_q(2), N_q);

limits_qdot = [-pi*8, pi*8]; resolution_qdot = 1e-1;
N_qdot = ceil(abs(limits_qdot(2)-limits_qdot(1))/resolution_qdot);
qdot_range = linspace(limits_qdot(1), limits_qdot(2), N_qdot);

%% Load the system
load('../Simulation_Continuous/Systems/UAV_n4', 'System');
S = System;
n = size(S.M([0;0;0]), 1); l = size(S.Psi([0;0;0]), 2);
m = size(S.F([0;0;0]), 2);

%% Define combined matrices
Fc = @(q) S.M(q)*S.Psi(q)*inv(S.Psi(q)'*S.Psi(q));
A = @(q) -inv(S.F(q)'*S.F(q))*S.F(q)'*Fc(q)*S.Psi(q)'*(...
    (S.lambda + S.gamma)*eye(3) + S.F(q)*S.Kv(q)*S.F(q)') - ...
    S.Kv(q)*S.F(q)';

B = @(q, qdot) -inv(S.F(q)'*S.F(q))*S.F(q)'*Fc(q)*S.Psi(q)'*(...
    S.M(q)*S.dMinvdt(q, qdot)*S.M(q) +...
    S.Psi(q)*inv(S.Psi(q)'*S.Psi(q))*S.dPsi(q, qdot)');

C = @(q) inv(S.F(q)'*S.F(q))*S.F(q)'*Fc(q);
D = @(q) inv(S.F(q)'*S.F(q))*S.F(q)'*(S.dHdq(q) - S.nPsi(q)*S.dVs(q));

%% Initialise results
LUT.A_table = zeros(l, n, N_q);
LUT.C_table = zeros(l, l, N_q);
LUT.D_table = zeros(m, N_q);
LUT.B_table = zeros(l, n, N_q, N_qdot);

%% Construct tables
bar = waitbar(i / numel(q_range), 'Constructing LUTs');
for i = 1:numel(q_range)
    LUT.A_table(:, :, i) = A([zeros(n-1, 1); q_range(i)]);
    LUT.C_table(:, :, i) = C([zeros(n-1, 1); q_range(i)]);
    LUT.D_table(:, i) = D([zeros(n-1, 1); q_range(i)]);
    for j = 1:numel(qdot_range)
        LUT.B_table(:, :, i, j) = B([zeros(n-1, 1); q_range(i)], [zeros(n-1, 1); qdot_range(j)]);
    end
    waitbar(i / numel(q_range), bar, 'Constructing LUTs');
end
close(bar);

%% Save results
LUT.N_q = N_q; LUT.q_low = limits_q(1); LUT.q_res = resolution_q;
LUT.N_qdot = N_qdot; LUT.qdot_low = limits_qdot(1); LUT.qdot_res = resolution_qdot;
save([filename '_LUT'], 'LUT');

