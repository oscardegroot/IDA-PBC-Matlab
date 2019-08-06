function [tau_k, tau_m, tau_fb, System] = Manipulator_OptControl(qk, pk, tau_ck, index, Ts)
% REMARKS -- 
% qk = q[k]
% qkp = q[k+1]
%   removed for efficiency
% Uses euler as estimation
% --
    %% Find qdot[k], q[k+1], tau[k] (IDAPBC)
    load(['Systems/Manipulator3_n' num2str(index)], 'System');
    qdot_k = System.Minv(qk) * pk;
    qkp = qk + qdot_k*Ts;
    tau_k = System.dHdq(qk, qdot_k) - System.Kv(qk, qdot_k) * qdot_k;       
    
    %tau_ck = getMom(System, qk, tau_ck);
    tau_m = tau_ck;
    
    %% r[k+1] is feedforward calculated here
    % r[k+1] = C + K*b[k] with b[k] an extra vector for r-passivity
    pkC = pk + (-System.dHdq(qk, qdot_k) + tau_k + System.Psi(qk)*tau_ck)*Ts;
    qdot_kpC = System.Minv(qkp)*pkC;
    zdot_kpC = System.Psi(qkp)'*qdot_kpC;
    z_kpC = System.a(qkp);
    C = zdot_kpC + System.lambda*z_kpC;
    K = System.Psi(qkp)'*System.Minv(qkp)*System.Psi(qk)*Ts;

    % Calculate r[k]
    rk = System.r(qk, qdot_k);
    
    %% Optimisation
    % Minimise the norm of vector b
    Q = eye(2);
   
    % Subject to discrete r-passivity
    H = 0.5*K'*K;
    k = K'*C;
    d = 0.5*(C'*C) - 0.5*rk'*rk - tau_ck'*rk;
    
    fun = @(b) deal(0.5*b'*Q*b, Q*b);
    constr = @(b) deal(b'*H*b + k'*b + d, [], 2*H*b+k, []);
    b0 = [0; 0]; % column vector
    
    options = optimoptions(@fmincon,'Algorithm','interior-point',...
    'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,...
    'HessianFcn', @(b, lambda) Q + lambda.ineqnonlin(1)*H, 'MaxIterations', 5000);
    
    [b_res,fval,eflag,output,lambda] = ...
        fmincon(fun,b0,[],[],[],[],[],[],constr,options);
    b_res
    %% Output
    tau_fb = tau_ck + b_res;
end
    %% OOOOOLD
    %r = System.r(q, qdot);
%     Aeq = -System.Psi(q)'*System.Minv(q);
%     Beq = -tau_c+System.Psi(q)'*System.Minv(q)*System.Psi(q)*tau_c - System.Psi(q)'*System.Minv(q)*(System.dV(q) + ...
%         0.5*System.qdotM(q, qdot)*qdot)+System.Psi(q)'*System.dMinvdt(q, qdot)*p + ...
%         System.dPsi(q, qdot)'*qdot + (System.lambda)*System.Psi(q)'*qdot;
%     [tau, fval] = quadprog(Aeq'*Aeq, -Beq'*Aeq, [], [],[],[],[],[],[], optimoptions('quadprog','Display','off', 'StepTolerance', 1e-3, 'OptimalityTolerance', 1e-3));
    %[tau, fval] = quadprog(Aeq'*r*r'*Aeq, -(Beq'*r - qdot'*qdot)*r'*Aeq, [], [],[],[],[],[],[], optimoptions('quadprog','Display','off', 'StepTolerance', 1e-5, 'OptimalityTolerance', 1e-5));

    % Add here a condition that guarantuees passivity! S >= 0!
    %tau = inv(Aeq'*Aeq+5e-1)*Aeq'*Beq
    %fprintf('%.2f\n', fval+0.5*(Beq'*r - qdot'*qdot)'*(Beq'*r - qdot'*qdot));
%     tau_fb = zeros(2, 1);
% end