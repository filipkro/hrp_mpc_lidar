% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(x_1 - r_1, Q) + quad_form(u_1 - u_0, R) + quad_form(x_2 - r_2, Q) + quad_form(u_2 - u_1, R) + quad_form(x_3 - r_3, Q) + quad_form(u_3 - u_2, R) + quad_form(x_4 - r_4, Q) + quad_form(u_4 - u_3, R) + quad_form(x_5 - r_5, Q) + quad_form(u_5 - u_4, R) + quad_form(x_6 - r_6, Q) + quad_form(u_6 - u_5, R) + quad_form(x_7 - r_7, Q) + quad_form(u_7 - u_6, R) + quad_form(x_8 - r_8, Q) + quad_form(u_8 - u_7, R) + quad_form(x_9 - r_9, Q) + quad_form(u_9 - u_8, R) + quad_form(x_10 - r_10, Q) + quad_form(u_10 - u_9, R) + quad_form(x_11 - r_11, Q_final) + quad_form(x_0 - r_0, Q))
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
%     abs(u_0(1)) <= u1_max
%     abs(u_1(1)) <= u1_max
%     abs(u_2(1)) <= u1_max
%     abs(u_3(1)) <= u1_max
%     abs(u_4(1)) <= u1_max
%     abs(u_5(1)) <= u1_max
%     abs(u_6(1)) <= u1_max
%     abs(u_7(1)) <= u1_max
%     abs(u_8(1)) <= u1_max
%     abs(u_9(1)) <= u1_max
%     abs(u_10(1)) <= u1_max
%     abs(u_0(2)) <= u2_max
%     abs(u_1(2)) <= u2_max
%     abs(u_2(2)) <= u2_max
%     abs(u_3(2)) <= u2_max
%     abs(u_4(2)) <= u2_max
%     abs(u_5(2)) <= u2_max
%     abs(u_6(2)) <= u2_max
%     abs(u_7(2)) <= u2_max
%     abs(u_8(2)) <= u2_max
%     abs(u_9(2)) <= u2_max
%     abs(u_10(2)) <= u2_max
%     abs(u_1(1) - u_0(1)) <= S
%     abs(u_2(1) - u_1(1)) <= S
%     abs(u_3(1) - u_2(1)) <= S
%     abs(u_4(1) - u_3(1)) <= S
%     abs(u_5(1) - u_4(1)) <= S
%     abs(u_6(1) - u_5(1)) <= S
%     abs(u_7(1) - u_6(1)) <= S
%     abs(u_8(1) - u_7(1)) <= S
%     abs(u_9(1) - u_8(1)) <= S
%     abs(u_10(1) - u_9(1)) <= S
%     abs(u_0(1) - u1_prev) <= S
%     abs(u_1(2) - u_0(2)) <= Th
%     abs(u_2(2) - u_1(2)) <= Th
%     abs(u_3(2) - u_2(2)) <= Th
%     abs(u_4(2) - u_3(2)) <= Th
%     abs(u_5(2) - u_4(2)) <= Th
%     abs(u_6(2) - u_5(2)) <= Th
%     abs(u_7(2) - u_6(2)) <= Th
%     abs(u_8(2) - u_7(2)) <= Th
%     abs(u_9(2) - u_8(2)) <= Th
%     abs(u_10(2) - u_9(2)) <= Th
%     abs(u_0(2) - u2_prev) <= Th
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
%      x_1   3 x 1
%      x_2   3 x 1
%      x_3   3 x 1
%      x_4   3 x 1
%      x_5   3 x 1
%      x_6   3 x 1
%      x_7   3 x 1
%      x_8   3 x 1
%      x_9   3 x 1
%     x_10   3 x 1
%     x_11   3 x 1
%
% and parameters
%        A   3 x 3
%        B   3 x 2
%        Q   3 x 3    PSD
%  Q_final   3 x 3    PSD
%        R   2 x 2    PSD
%        S   1 x 1    positive
%       Th   1 x 1    positive
%      r_0   3 x 1
%      r_1   3 x 1
%      r_2   3 x 1
%      r_3   3 x 1
%      r_4   3 x 1
%      r_5   3 x 1
%      r_6   3 x 1
%      r_7   3 x 1
%      r_8   3 x 1
%      r_9   3 x 1
%     r_10   3 x 1
%     r_11   3 x 1
%   u1_max   1 x 1    positive
%  u1_prev   1 x 1
%   u2_max   1 x 1    positive
%  u2_prev   1 x 1
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
% Produced by CVXGEN, 2019-04-15 05:31:41 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
