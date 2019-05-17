params.A = eye(3);
params.B = [1 0; 1 0; 0 1];
params.Q = eye(3);
params.Q_final = eye(3);
params.R = eye(2);
params.x_0 = [0;0;0];
params.u_max = 5;
params.S = 5;
params.r_0 = [0;0;0];
for i = 1:11
    params.r{i} = [i;i;0.78];
end

% Exercise the high-speed solver.
[vars, status] = csolve(params);  % solve, saving results.

% Check convergence, and display the optimal variable value.
if ~status.converged, error 'failed to converge'; end
vars.x