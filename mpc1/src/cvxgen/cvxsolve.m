% Produced by CVXGEN, 2019-05-18 09:53:58 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: cvxsolve.m.
% Description: Solution file, via cvx, for use with sample.m.
function [vars, status] = cvxsolve(params, settings)
A = params.A;
B = params.B;
Q = params.Q;
R = params.R;
deltau_max = params.deltau_max;
if isfield(params, 'r_1')
  r_1 = params.r_1;
elseif isfield(params, 'r')
  r_1 = params.r{1};
else
  error 'could not find r_1'
end
if isfield(params, 'r_2')
  r_2 = params.r_2;
elseif isfield(params, 'r')
  r_2 = params.r{2};
else
  error 'could not find r_2'
end
if isfield(params, 'r_3')
  r_3 = params.r_3;
elseif isfield(params, 'r')
  r_3 = params.r{3};
else
  error 'could not find r_3'
end
if isfield(params, 'r_4')
  r_4 = params.r_4;
elseif isfield(params, 'r')
  r_4 = params.r{4};
else
  error 'could not find r_4'
end
if isfield(params, 'r_5')
  r_5 = params.r_5;
elseif isfield(params, 'r')
  r_5 = params.r{5};
else
  error 'could not find r_5'
end
u_max = params.u_max;
u_prev = params.u_prev;
x_0 = params.x_0;
cvx_begin
  % Caution: automatically generated by cvxgen. May be incorrect.
  variable x_1(3, 1);
  variable u_1(2, 1);
  variable u_0(2, 1);
  variable x_2(3, 1);
  variable u_2(2, 1);
  variable x_3(3, 1);
  variable u_3(2, 1);
  variable x_4(3, 1);
  variable u_4(2, 1);
  variable x_5(3, 1);
  variable u_5(2, 1);
  variable x_6(3, 1);

  minimize(quad_form(x_1 - r_1, Q) + quad_form(u_1 - u_0, R) + quad_form(x_2 - r_2, Q) + quad_form(u_2 - u_1, R) + quad_form(x_3 - r_3, Q) + quad_form(u_3 - u_2, R) + quad_form(x_4 - r_4, Q) + quad_form(u_4 - u_3, R) + quad_form(x_5 - r_5, Q) + quad_form(u_5 - u_4, R) + quad_form(u_0 - u_prev, R));
  subject to
    x_1 == A*x_0 + B*u_0;
    x_2 == A*x_1 + B*u_1;
    x_3 == A*x_2 + B*u_2;
    x_4 == A*x_3 + B*u_3;
    x_5 == A*x_4 + B*u_4;
    x_6 == A*x_5 + B*u_5;
    u_0(1) <= u_max(1);
    u_1(1) <= u_max(1);
    u_2(1) <= u_max(1);
    u_3(1) <= u_max(1);
    u_4(1) <= u_max(1);
    u_5(1) <= u_max(1);
    u_0(1) >= 0;
    u_1(1) >= 0;
    u_2(1) >= 0;
    u_3(1) >= 0;
    u_4(1) >= 0;
    u_5(1) >= 0;
    abs(u_0(2)) <= u_max(2);
    abs(u_1(2)) <= u_max(2);
    abs(u_2(2)) <= u_max(2);
    abs(u_3(2)) <= u_max(2);
    abs(u_4(2)) <= u_max(2);
    abs(u_5(2)) <= u_max(2);
    abs(u_1 - u_0) <= deltau_max;
    abs(u_2 - u_1) <= deltau_max;
    abs(u_3 - u_2) <= deltau_max;
    abs(u_4 - u_3) <= deltau_max;
    abs(u_5 - u_4) <= deltau_max;
    abs(u_1 - u_prev) <= deltau_max;
    abs(u_1 - u_prev) <= deltau_max;
    abs(u_1 - u_prev) <= deltau_max;
    abs(u_1 - u_prev) <= deltau_max;
    abs(u_1 - u_prev) <= deltau_max;
cvx_end
vars.u_0 = u_0;
vars.u_1 = u_1;
vars.u{1} = u_1;
vars.u_2 = u_2;
vars.u{2} = u_2;
vars.u_3 = u_3;
vars.u{3} = u_3;
vars.u_4 = u_4;
vars.u{4} = u_4;
vars.u_5 = u_5;
vars.u{5} = u_5;
vars.x_1 = x_1;
vars.x{1} = x_1;
vars.x_2 = x_2;
vars.x{2} = x_2;
vars.x_3 = x_3;
vars.x{3} = x_3;
vars.x_4 = x_4;
vars.x{4} = x_4;
vars.x_5 = x_5;
vars.x{5} = x_5;
vars.x_6 = x_6;
vars.x{6} = x_6;
status.cvx_status = cvx_status;
% Provide a drop-in replacement for csolve.
status.optval = cvx_optval;
status.converged = strcmp(cvx_status, 'Solved');
