% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(x_1 - r_1, Q) + quad_form(u_1 - u_0, R) + quad_form(x_2 - r_2, Q) + quad_form(u_2 - u_1, R) + quad_form(x_3 - r_3, Q) + quad_form(u_3 - u_2, R) + quad_form(x_4 - r_4, Q) + quad_form(u_4 - u_3, R) + quad_form(x_5 - r_5, Q) + quad_form(u_5 - u_4, R) + quad_form(u_0 - u_prev, R))
%   subject to
%     x_1 == A*x_0 + B*u_0
%     x_2 == A*x_1 + B*u_1
%     x_3 == A*x_2 + B*u_2
%     x_4 == A*x_3 + B*u_3
%     x_5 == A*x_4 + B*u_4
%     x_6 == A*x_5 + B*u_5
%     u_0(1) <= u_max(1)
%     u_1(1) <= u_max(1)
%     u_2(1) <= u_max(1)
%     u_3(1) <= u_max(1)
%     u_4(1) <= u_max(1)
%     u_5(1) <= u_max(1)
%     u_0(1) >= 0
%     u_1(1) >= 0
%     u_2(1) >= 0
%     u_3(1) >= 0
%     u_4(1) >= 0
%     u_5(1) >= 0
%     abs(u_0(2)) <= u_max(2)
%     abs(u_1(2)) <= u_max(2)
%     abs(u_2(2)) <= u_max(2)
%     abs(u_3(2)) <= u_max(2)
%     abs(u_4(2)) <= u_max(2)
%     abs(u_5(2)) <= u_max(2)
%     abs(u_1 - u_0) <= deltau_max
%     abs(u_2 - u_1) <= deltau_max
%     abs(u_3 - u_2) <= deltau_max
%     abs(u_4 - u_3) <= deltau_max
%     abs(u_5 - u_4) <= deltau_max
%     abs(u_1 - u_prev) <= deltau_max
%     abs(u_1 - u_prev) <= deltau_max
%     abs(u_1 - u_prev) <= deltau_max
%     abs(u_1 - u_prev) <= deltau_max
%     abs(u_1 - u_prev) <= deltau_max
%
% with variables
%      u_0   2 x 1
%      u_1   2 x 1
%      u_2   2 x 1
%      u_3   2 x 1
%      u_4   2 x 1
%      u_5   2 x 1
%      x_1   3 x 1
%      x_2   3 x 1
%      x_3   3 x 1
%      x_4   3 x 1
%      x_5   3 x 1
%      x_6   3 x 1
%
% and parameters
%        A   3 x 3
%        B   3 x 2
%        Q   3 x 3    PSD
%        R   2 x 2    PSD
% deltau_max   2 x 1    positive
%      r_1   3 x 1
%      r_2   3 x 1
%      r_3   3 x 1
%      r_4   3 x 1
%      r_5   3 x 1
%    u_max   2 x 1    positive
%   u_prev   2 x 1
%      x_0   3 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.x_0, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2019-05-18 09:53:58 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
