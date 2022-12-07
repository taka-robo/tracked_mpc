% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(x_1 - x_ref_1, Q) + quad_form(u_1 - u_ref_1, R) + quad_form(x_2 - x_ref_2, Q) + quad_form(u_2 - u_ref_2, R) + quad_form(x_3 - x_ref_3, Q) + quad_form(u_3 - u_ref_3, R) + quad_form(x_4 - x_ref_4, Q) + quad_form(u_4 - u_ref_4, R) + quad_form(x_5 - x_ref_5, Q) + quad_form(u_5 - u_ref_5, R) + quad_form(x_6 - x_ref_6, Q) + quad_form(u_6 - u_ref_6, R) + quad_form(x_7 - x_ref_7, Q) + quad_form(u_7 - u_ref_7, R) + quad_form(x_8 - x_ref_8, Q) + quad_form(u_8 - u_ref_8, R) + quad_form(x_9 - x_ref_9, Q) + quad_form(u_9 - u_ref_9, R) + quad_form(x_10 - x_ref_10, Q) + quad_form(u_10 - u_ref_10, R))
%   subject to
%     x_1 == A*x_0 + B*u_0 + w_0
%     x_2 == A*x_1 + B*u_1 + w_1
%     x_3 == A*x_2 + B*u_2 + w_2
%     x_4 == A*x_3 + B*u_3 + w_3
%     x_5 == A*x_4 + B*u_4 + w_4
%     x_6 == A*x_5 + B*u_5 + w_5
%     x_7 == A*x_6 + B*u_6 + w_6
%     x_8 == A*x_7 + B*u_7 + w_7
%     x_9 == A*x_8 + B*u_8 + w_8
%     x_10 == A*x_9 + B*u_9 + w_9
%     x_11 == A*x_10 + B*u_10 + w_10
%     u_1 <= u_max
%     u_2 <= u_max
%     u_3 <= u_max
%     u_4 <= u_max
%     u_5 <= u_max
%     u_6 <= u_max
%     u_7 <= u_max
%     u_8 <= u_max
%     u_9 <= u_max
%     u_10 <= u_max
%
% with variables
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
%      x_1   2 x 1
%      x_2   2 x 1
%      x_3   2 x 1
%      x_4   2 x 1
%      x_5   2 x 1
%      x_6   2 x 1
%      x_7   2 x 1
%      x_8   2 x 1
%      x_9   2 x 1
%     x_10   2 x 1
%     x_11   2 x 1
%
% and parameters
%        A   2 x 2
%        B   2 x 2
%        Q   2 x 2    PSD
%        R   2 x 2    PSD
%      u_0   2 x 1
%    u_max   2 x 1    positive
%  u_ref_1   2 x 1
%  u_ref_2   2 x 1
%  u_ref_3   2 x 1
%  u_ref_4   2 x 1
%  u_ref_5   2 x 1
%  u_ref_6   2 x 1
%  u_ref_7   2 x 1
%  u_ref_8   2 x 1
%  u_ref_9   2 x 1
% u_ref_10   2 x 1
%      w_0   2 x 1
%      w_1   2 x 1
%      w_2   2 x 1
%      w_3   2 x 1
%      w_4   2 x 1
%      w_5   2 x 1
%      w_6   2 x 1
%      w_7   2 x 1
%      w_8   2 x 1
%      w_9   2 x 1
%     w_10   2 x 1
%      x_0   2 x 1
%  x_ref_1   2 x 1
%  x_ref_2   2 x 1
%  x_ref_3   2 x 1
%  x_ref_4   2 x 1
%  x_ref_5   2 x 1
%  x_ref_6   2 x 1
%  x_ref_7   2 x 1
%  x_ref_8   2 x 1
%  x_ref_9   2 x 1
% x_ref_10   2 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.x_ref_10, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2022-12-07 11:01:31 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
