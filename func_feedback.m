% Compute control action using feedback linearization
%
% Inputs:
%       x: states
%           q1
%           q2
%           q3
%           dq1
%           dq2
%           dq3
%       alpha: Bezier coefficients for q2 and q3
%           alpha2 (1st to 5th coefficients)
%           alpha3 (1st to 5th coefficients)
%       s_params: for gait timing
%           q1_min
%           delq
%
% Outputs:
%       u: control action
%
function u = func_feedback(x,alpha,s_params)
% gains
kp1 = 2500;
kp2 = 2500;
kd1 = 500;
kd2 = 500;
% Seperating inputs
q = x(1:3);  q = q(:);
dq = x(4:6); dq = dq(:);
% Get model parameters
[r,m,Mh,Mt,l,g] = func_model_params;
params = [r,m,Mh,Mt,l,g];
% Seperate Bezier coefficients
alpha2 = alpha(1:5);
alpha3 = alpha(6:10);
% Seperate s_params (template convention: s_params = [q1_min (post-impact), q1_max (pre-impact)])
q1_min = s_params(1);
q1_max = s_params(2);
delq = q1_max - q1_min;
% Gait timing variable
s = func_gait_timing(q(1), q1_min, q1_max);
% Saturate s to [0,1] to prevent Bezier extrapolation
if s > 1
    s = 1;
elseif s < 0
    s = 0;
end
% Get D,C,G,B matrices
[D,C,G,B] = func_compute_D_C_G_B(q,dq,params);
% Defining fx and gx (from xdot = f(x) + g(x)*u)
fx = [dq; D\(-C*dq - G)];
gx = [zeros(3,2); D\B];
% find y = h(x) = q_b - b(s)
b2 = bezier(s,4,alpha2);
b3 = bezier(s,4,alpha3);
h = [q(2) - b2; q(3) - b3];
y = h;
% Calculating y_dot = Lfh = dh/dx * fx
% h is a function of (s, q2, q3, dq), not q1 directly.
% Use chain rule: dh/dq1 = dh/ds * ds/dq1 = dh/ds * (1/delq)
db_ds2 = d_ds_bezier(s,4,alpha2);
db_ds3 = d_ds_bezier(s,4,alpha3);
% dh_dx is 2x6: [dh/dq1, dh/dq2, dh/dq3, dh/ddq1, dh/ddq2, dh/ddq3]
dh_dx = zeros(2,6);
dh_dx(1,1) = -db_ds2/delq;   % dh1/dq1 = -db2/ds * 1/delq
dh_dx(2,1) = -db_ds3/delq;   % dh2/dq1 = -db3/ds * 1/delq
dh_dx(1,2) = 1;              % dh1/dq2 = 1
dh_dx(2,3) = 1;              % dh2/dq3 = 1
Lfh = dh_dx*fx;
dy = Lfh;
%%%% PD controller
Kp = [kp1,0; 0,kp2];
Kd = [kd1,0; 0,kd2];
v = -Kp*y - Kd*dy;
%%%% Feedback linearization
% dLfh is the 2x6 jacobian of Lfh w.r.t. [q; dq] (with 1/delq on q1 column)
dLfh = func_compute_dLfh([s,delq],dq(1),[alpha2,alpha3]);
L2fh = dLfh*fx;
LgLfh = dLfh*gx;
% Control action: u = LgLfh^-1 * (v - L2fh)
u = LgLfh\(v - L2fh);
end