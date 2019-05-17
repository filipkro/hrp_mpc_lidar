% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(x_1 - r_1, Q) + quad_form(u_1 - u_0, R) + quad_form(x_2 - r_2, Q) + quad_form(u_2 - u_1, R) + quad_form(x_3 - r_3, Q) + quad_form(u_3 - u_2, R) + quad_form(x_4 - r_4, Q) + quad_form(u_4 - u_3, R) + quad_form(x_5 - r_5, Q) + quad_form(u_5 - u_4, R) + quad_form(x_6 - r_6, Q) + quad_form(u_6 - u_5, R) + quad_form(x_7 - r_7, Q) + quad_form(u_7 - u_6, R) + quad_form(x_8 - r_8, Q) + quad_form(u_8 - u_7, R) + quad_form(x_9 - r_9, Q) + quad_form(u_9 - u_8, R) + quad_form(x_10 - r_10, Q) + quad_form(u_10 - u_9, R) + quad_form(x_11 - r_11, Q_final) + quad_form(x_0 - r_0, Q) + quad_form(u_0 - u_prev, eye(2)))
%   subject to
%     x_1 == A*x_0 + B*u_0
%     x_2 == A*x_1 + B*u_1
%     x_3 == A*x_2 + B*u_2
%     x_4 == A*x_3 + B*u_3
%     x_5 == A*x_4 + B*u_4
%     x_6 == A*x_5 + B*u_5
%     x_7 == A*x_6 + B*u_6
%     x_8 == A*x_7 + B*u_7
%     x_9 == A*x_8 + B*u_8
%     x_10 == A*x_9 + B*u_9
%     x_11 == A*x_10 + B*u_10
%     abs(u_0(1)) <= u_max(1)
%     abs(u_1(1)) <= u_max(1)
%     abs(u_2(1)) <= u_max(1)
%     abs(u_3(1)) <= u_max(1)
%     abs(u_4(1)) <= u_max(1)
%     abs(u_5(1)) <= u_max(1)
%     abs(u_6(1)) <= u_max(1)
%     abs(u_7(1)) <= u_max(1)
%     abs(u_8(1)) <= u_max(1)
%     abs(u_9(1)) <= u_max(1)
%     abs(u_10(1)) <= u_max(1)
%     abs(u_0(2)) <= u_max(2)
%     abs(u_1(2)) <= u_max(2)
%     abs(u_2(2)) <= u_max(2)
%     abs(u_3(2)) <= u_max(2)
%     abs(u_4(2)) <= u_max(2)
%     abs(u_5(2)) <= u_max(2)
%     abs(u_6(2)) <= u_max(2)
%     abs(u_7(2)) <= u_max(2)
%     abs(u_8(2)) <= u_max(2)
%     abs(u_9(2)) <= u_max(2)
%     abs(u_10(2)) <= u_max(2)
%     abs(u_1(1) - u_0(1)) <= deltau_max(1)
%     abs(u_2(1) - u_1(1)) <= deltau_max(1)
%     abs(u_3(1) - u_2(1)) <= deltau_max(1)
%     abs(u_4(1) - u_3(1)) <= deltau_max(1)
%     abs(u_5(1) - u_4(1)) <= deltau_max(1)
%     abs(u_6(1) - u_5(1)) <= deltau_max(1)
%     abs(u_7(1) - u_6(1)) <= deltau_max(1)
%     abs(u_8(1) - u_7(1)) <= deltau_max(1)
%     abs(u_9(1) - u_8(1)) <= deltau_max(1)
%     abs(u_10(1) - u_9(1)) <= deltau_max(1)
%     abs(u_0(1) - u_prev(1)) <= deltau_max(1)
%     abs(u_1(2) - u_0(2)) <= deltau_max(2)
%     abs(u_2(2) - u_1(2)) <= deltau_max(2)
%     abs(u_3(2) - u_2(2)) <= deltau_max(2)
%     abs(u_4(2) - u_3(2)) <= deltau_max(2)
%     abs(u_5(2) - u_4(2)) <= deltau_max(2)
%     abs(u_6(2) - u_5(2)) <= deltau_max(2)
%     abs(u_7(2) - u_6(2)) <= deltau_max(2)
%     abs(u_8(2) - u_7(2)) <= deltau_max(2)
%     abs(u_9(2) - u_8(2)) <= deltau_max(2)
%     abs(u_10(2) - u_9(2)) <= deltau_max(2)
%     abs(u_0(2) - u_prev(2)) <= deltau_max(2)
%     abs(x_1(4)) <= dist(1)
%     abs(x_2(4)) <= dist(1)
%     abs(x_3(4)) <= dist(1)
%     abs(x_4(4)) <= dist(1)
%     abs(x_5(4)) <= dist(1)
%     abs(x_6(4)) <= dist(1)
%     abs(x_7(4)) <= dist(1)
%     abs(x_8(4)) <= dist(1)
%     abs(x_9(4)) <= dist(1)
%     abs(x_10(4)) <= dist(1)
%     abs(x_1(5)) <= dist(2)
%     abs(x_2(5)) <= dist(2)
%     abs(x_3(5)) <= dist(2)
%     abs(x_4(5)) <= dist(2)
%     abs(x_5(5)) <= dist(2)
%     abs(x_6(5)) <= dist(2)
%     abs(x_7(5)) <= dist(2)
%     abs(x_8(5)) <= dist(2)
%     abs(x_9(5)) <= dist(2)
%     abs(x_10(5)) <= dist(2)
%
% with variables
%      u_0   2 x 1
%      u_1   2 x 1
%      u_2   2 x 1
%      u_3   2 x 1
%      u_4   2 x 1
%      u_5   2 x 1
%      u_6   2 x 1
%      u_7   2 x 1
%      u_8   2 x 1
%      u_9   2 x 1
%     u_10   2 x 1
%      x_1   5 x 1
%      x_2   5 x 1
%      x_3   5 x 1
%      x_4   5 x 1
%      x_5   5 x 1
%      x_6   5 x 1
%      x_7   5 x 1
%      x_8   5 x 1
%      x_9   5 x 1
%     x_10   5 x 1
%     x_11   5 x 1
%
% and parameters
%        A   5 x 5
%        B   5 x 2
%        Q   5 x 5    PSD
%  Q_final   5 x 5    PSD
%        R   2 x 2    PSD
% deltau_max   2 x 1    positive
%     dist   2 x 1
%      r_0   5 x 1
%      r_1   5 x 1
%      r_2   5 x 1
%      r_3   5 x 1
%      r_4   5 x 1
%      r_5   5 x 1
%      r_6   5 x 1
%      r_7   5 x 1
%      r_8   5 x 1
%      r_9   5 x 1
%     r_10   5 x 1
%     r_11   5 x 1
%    u_max   2 x 1    positive
%   u_prev   2 x 1
%      x_0   5 x 1
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
% Produced by CVXGEN, 2019-04-16 07:32:27 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
