% Produced by CVXGEN, 2022-12-07 11:01:31 -0500.
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
u_0 = params.u_0;
u_max = params.u_max;
if isfield(params, 'u_ref_1')
  u_ref_1 = params.u_ref_1;
elseif isfield(params, 'u_ref')
  u_ref_1 = params.u_ref{1};
else
  error 'could not find u_ref_1'
end
if isfield(params, 'u_ref_2')
  u_ref_2 = params.u_ref_2;
elseif isfield(params, 'u_ref')
  u_ref_2 = params.u_ref{2};
else
  error 'could not find u_ref_2'
end
if isfield(params, 'u_ref_3')
  u_ref_3 = params.u_ref_3;
elseif isfield(params, 'u_ref')
  u_ref_3 = params.u_ref{3};
else
  error 'could not find u_ref_3'
end
if isfield(params, 'u_ref_4')
  u_ref_4 = params.u_ref_4;
elseif isfield(params, 'u_ref')
  u_ref_4 = params.u_ref{4};
else
  error 'could not find u_ref_4'
end
if isfield(params, 'u_ref_5')
  u_ref_5 = params.u_ref_5;
elseif isfield(params, 'u_ref')
  u_ref_5 = params.u_ref{5};
else
  error 'could not find u_ref_5'
end
if isfield(params, 'u_ref_6')
  u_ref_6 = params.u_ref_6;
elseif isfield(params, 'u_ref')
  u_ref_6 = params.u_ref{6};
else
  error 'could not find u_ref_6'
end
if isfield(params, 'u_ref_7')
  u_ref_7 = params.u_ref_7;
elseif isfield(params, 'u_ref')
  u_ref_7 = params.u_ref{7};
else
  error 'could not find u_ref_7'
end
if isfield(params, 'u_ref_8')
  u_ref_8 = params.u_ref_8;
elseif isfield(params, 'u_ref')
  u_ref_8 = params.u_ref{8};
else
  error 'could not find u_ref_8'
end
if isfield(params, 'u_ref_9')
  u_ref_9 = params.u_ref_9;
elseif isfield(params, 'u_ref')
  u_ref_9 = params.u_ref{9};
else
  error 'could not find u_ref_9'
end
if isfield(params, 'u_ref_10')
  u_ref_10 = params.u_ref_10;
elseif isfield(params, 'u_ref')
  u_ref_10 = params.u_ref{10};
else
  error 'could not find u_ref_10'
end
w_0 = params.w_0;
if isfield(params, 'w_1')
  w_1 = params.w_1;
elseif isfield(params, 'w')
  w_1 = params.w{1};
else
  error 'could not find w_1'
end
if isfield(params, 'w_2')
  w_2 = params.w_2;
elseif isfield(params, 'w')
  w_2 = params.w{2};
else
  error 'could not find w_2'
end
if isfield(params, 'w_3')
  w_3 = params.w_3;
elseif isfield(params, 'w')
  w_3 = params.w{3};
else
  error 'could not find w_3'
end
if isfield(params, 'w_4')
  w_4 = params.w_4;
elseif isfield(params, 'w')
  w_4 = params.w{4};
else
  error 'could not find w_4'
end
if isfield(params, 'w_5')
  w_5 = params.w_5;
elseif isfield(params, 'w')
  w_5 = params.w{5};
else
  error 'could not find w_5'
end
if isfield(params, 'w_6')
  w_6 = params.w_6;
elseif isfield(params, 'w')
  w_6 = params.w{6};
else
  error 'could not find w_6'
end
if isfield(params, 'w_7')
  w_7 = params.w_7;
elseif isfield(params, 'w')
  w_7 = params.w{7};
else
  error 'could not find w_7'
end
if isfield(params, 'w_8')
  w_8 = params.w_8;
elseif isfield(params, 'w')
  w_8 = params.w{8};
else
  error 'could not find w_8'
end
if isfield(params, 'w_9')
  w_9 = params.w_9;
elseif isfield(params, 'w')
  w_9 = params.w{9};
else
  error 'could not find w_9'
end
if isfield(params, 'w_10')
  w_10 = params.w_10;
elseif isfield(params, 'w')
  w_10 = params.w{10};
else
  error 'could not find w_10'
end
x_0 = params.x_0;
if isfield(params, 'x_ref_1')
  x_ref_1 = params.x_ref_1;
elseif isfield(params, 'x_ref')
  x_ref_1 = params.x_ref{1};
else
  error 'could not find x_ref_1'
end
if isfield(params, 'x_ref_2')
  x_ref_2 = params.x_ref_2;
elseif isfield(params, 'x_ref')
  x_ref_2 = params.x_ref{2};
else
  error 'could not find x_ref_2'
end
if isfield(params, 'x_ref_3')
  x_ref_3 = params.x_ref_3;
elseif isfield(params, 'x_ref')
  x_ref_3 = params.x_ref{3};
else
  error 'could not find x_ref_3'
end
if isfield(params, 'x_ref_4')
  x_ref_4 = params.x_ref_4;
elseif isfield(params, 'x_ref')
  x_ref_4 = params.x_ref{4};
else
  error 'could not find x_ref_4'
end
if isfield(params, 'x_ref_5')
  x_ref_5 = params.x_ref_5;
elseif isfield(params, 'x_ref')
  x_ref_5 = params.x_ref{5};
else
  error 'could not find x_ref_5'
end
if isfield(params, 'x_ref_6')
  x_ref_6 = params.x_ref_6;
elseif isfield(params, 'x_ref')
  x_ref_6 = params.x_ref{6};
else
  error 'could not find x_ref_6'
end
if isfield(params, 'x_ref_7')
  x_ref_7 = params.x_ref_7;
elseif isfield(params, 'x_ref')
  x_ref_7 = params.x_ref{7};
else
  error 'could not find x_ref_7'
end
if isfield(params, 'x_ref_8')
  x_ref_8 = params.x_ref_8;
elseif isfield(params, 'x_ref')
  x_ref_8 = params.x_ref{8};
else
  error 'could not find x_ref_8'
end
if isfield(params, 'x_ref_9')
  x_ref_9 = params.x_ref_9;
elseif isfield(params, 'x_ref')
  x_ref_9 = params.x_ref{9};
else
  error 'could not find x_ref_9'
end
if isfield(params, 'x_ref_10')
  x_ref_10 = params.x_ref_10;
elseif isfield(params, 'x_ref')
  x_ref_10 = params.x_ref{10};
else
  error 'could not find x_ref_10'
end
cvx_begin
  % Caution: automatically generated by cvxgen. May be incorrect.
  variable x_1(2, 1);
  variable u_1(2, 1);
  variable x_2(2, 1);
  variable u_2(2, 1);
  variable x_3(2, 1);
  variable u_3(2, 1);
  variable x_4(2, 1);
  variable u_4(2, 1);
  variable x_5(2, 1);
  variable u_5(2, 1);
  variable x_6(2, 1);
  variable u_6(2, 1);
  variable x_7(2, 1);
  variable u_7(2, 1);
  variable x_8(2, 1);
  variable u_8(2, 1);
  variable x_9(2, 1);
  variable u_9(2, 1);
  variable x_10(2, 1);
  variable u_10(2, 1);
  variable x_11(2, 1);

  minimize(quad_form(x_1 - x_ref_1, Q) + quad_form(u_1 - u_ref_1, R) + quad_form(x_2 - x_ref_2, Q) + quad_form(u_2 - u_ref_2, R) + quad_form(x_3 - x_ref_3, Q) + quad_form(u_3 - u_ref_3, R) + quad_form(x_4 - x_ref_4, Q) + quad_form(u_4 - u_ref_4, R) + quad_form(x_5 - x_ref_5, Q) + quad_form(u_5 - u_ref_5, R) + quad_form(x_6 - x_ref_6, Q) + quad_form(u_6 - u_ref_6, R) + quad_form(x_7 - x_ref_7, Q) + quad_form(u_7 - u_ref_7, R) + quad_form(x_8 - x_ref_8, Q) + quad_form(u_8 - u_ref_8, R) + quad_form(x_9 - x_ref_9, Q) + quad_form(u_9 - u_ref_9, R) + quad_form(x_10 - x_ref_10, Q) + quad_form(u_10 - u_ref_10, R));
  subject to
    x_1 == A*x_0 + B*u_0 + w_0;
    x_2 == A*x_1 + B*u_1 + w_1;
    x_3 == A*x_2 + B*u_2 + w_2;
    x_4 == A*x_3 + B*u_3 + w_3;
    x_5 == A*x_4 + B*u_4 + w_4;
    x_6 == A*x_5 + B*u_5 + w_5;
    x_7 == A*x_6 + B*u_6 + w_6;
    x_8 == A*x_7 + B*u_7 + w_7;
    x_9 == A*x_8 + B*u_8 + w_8;
    x_10 == A*x_9 + B*u_9 + w_9;
    x_11 == A*x_10 + B*u_10 + w_10;
    u_1 <= u_max;
    u_2 <= u_max;
    u_3 <= u_max;
    u_4 <= u_max;
    u_5 <= u_max;
    u_6 <= u_max;
    u_7 <= u_max;
    u_8 <= u_max;
    u_9 <= u_max;
    u_10 <= u_max;
cvx_end
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
vars.u_6 = u_6;
vars.u{6} = u_6;
vars.u_7 = u_7;
vars.u{7} = u_7;
vars.u_8 = u_8;
vars.u{8} = u_8;
vars.u_9 = u_9;
vars.u{9} = u_9;
vars.u_10 = u_10;
vars.u{10} = u_10;
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
vars.x_7 = x_7;
vars.x{7} = x_7;
vars.x_8 = x_8;
vars.x{8} = x_8;
vars.x_9 = x_9;
vars.x{9} = x_9;
vars.x_10 = x_10;
vars.x{10} = x_10;
vars.x_11 = x_11;
vars.x{11} = x_11;
status.cvx_status = cvx_status;
% Provide a drop-in replacement for csolve.
status.optval = cvx_optval;
status.converged = strcmp(cvx_status, 'Solved');
