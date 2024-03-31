clear all;
global poly_order
poly_order = 7;
c0 = rand(1, poly_order * 9);
filename = "way_point";

phase = 2 * pi * [0, 1/3, 2/3];
w1_x = 1.5 * cos(phase);
w1_y = 1.5 * sin(phase);

w2_x = 0.3 * cos(phase + pi / 3);
w2_y = 0.3 * sin(phase + pi / 3);
global wayp_x;
global wayp_y;
wayp_x = [w1_x(1), w2_x(1), w1_x(2), w2_x(2), w1_x(3), w2_x(3)];
wayp_y = [w1_y(1), w2_y(1), w1_y(2), w2_y(2), w1_y(3), w2_y(3)];


A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];

% Setting the optimization parameters.
options = optimoptions('fmincon','Algorithm','sqp','Display','iter');

global parameterValuesAtEachIteration;
parameterValuesAtEachIteration = [];

global cx;
global cy;
global cz;
% global cpsi;

syms cx   [1,poly_order];
syms cy   [1,poly_order];
syms cz   [1,poly_order];
% syms cpsi [1,poly_order];

global t;
syms t;

global sigma_x;
global sigma_y;
global sigma_z;
global dsigma_psi;

sigma_x = poly2sym(cx, t);
sigma_y = poly2sym(cy, t);
sigma_z = poly2sym(cz, t);
% sigma_psi = poly2sym(cpsi, t);

mu_p = 1;
mu_psi = 1;


global T;
T = 4;


global dsigma_x;
global dsigma_y;
global dsigma_z;


dsigma_x = diff(sigma_x,t,1);
dsigma_y = diff(sigma_y,t,1);
dsigma_z = diff(sigma_z,t,1);
sigma_psi = atan2(dsigma_y, dsigma_x);
dsigma_psi = diff(sigma_psi,t,1);

global d2sigma_x;
global d2sigma_y;
global d2sigma_z;

d2sigma_x = diff(sigma_x,t,2);
d2sigma_y = diff(sigma_y,t,2);
d2sigma_z = diff(sigma_z,t,2);


d4sigma_x(t) = diff(sigma_x,t,4);
d4sigma_y(t) = diff(sigma_y,t,4);
d4sigma_z(t) = diff(sigma_z,t,4);
% d4sigma_psi(t) = diff(sigma_psi,t,4);

m_p = 0.05;

sumSquare(t) = mu_p * d4sigma_x(t) ^ 2 + m_p * dsigma_x ^ 2 + ...
               mu_p * d4sigma_y(t) ^ 2 + m_p * dsigma_y ^ 2 +...
               mu_p * d4sigma_z(t) ^ 2 + m_p * dsigma_z ^ 2;
%                mu_psi * d4sigma_psi(t) ^ 2;

global intSumSquare;
intSumSquare = int (sumSquare(t), t, 0, T);

[coef,fval] = fmincon(@objfun,c0,A,b,Aeq,beq,lb,ub,@confuneq,options);

save (filename + ".mat","coef","T")


function [c,ceq] = confuneq(coef)
global t
% Nonlinear inequality constraints
c = [];
global wayp_x;
global wayp_y;

% Nonlinear equality constraints
global sigma_x;
global sigma_y;
global sigma_z;

% 
global dsigma_x;
global dsigma_y;
global dsigma_z;
global dsigma_psi;

global d2sigma_x;
global d2sigma_y;
global d2sigma_z;

%
global cx;
global cy;
global cz;
global poly_order;
% global cpsi;
% 
Csymbols = num2cell([cx,cy,cz]);

% first traj
Ccoef = num2cell(coef(1 : poly_order * 3));

pos_x = subs(sigma_x, Csymbols, Ccoef);
pos_y = subs(sigma_y, Csymbols, Ccoef);
pos_z = subs(sigma_z, Csymbols, Ccoef);

vel_x = subs(dsigma_x, Csymbols, Ccoef);
vel_y = subs(dsigma_y, Csymbols, Ccoef);
vel_z = subs(dsigma_z, Csymbols, Ccoef);
vel_psi = subs(dsigma_psi, Csymbols, Ccoef);
% 
acc_x = subs(d2sigma_x, Csymbols, Ccoef);
acc_y = subs(d2sigma_y, Csymbols, Ccoef);
acc_z = subs(d2sigma_z, Csymbols, Ccoef);
% second traj
Ccoef1 = num2cell(coef(poly_order * 3 + 1 : poly_order * 6));

pos_x1 = subs(sigma_x, Csymbols, Ccoef1);
pos_y1 = subs(sigma_y, Csymbols, Ccoef1);
pos_z1 = subs(sigma_z, Csymbols, Ccoef1);

vel_x1 = subs(dsigma_x, Csymbols, Ccoef1);
vel_y1 = subs(dsigma_y, Csymbols, Ccoef1);
vel_z1 = subs(dsigma_z, Csymbols, Ccoef1);
vel_psi1 = subs(dsigma_psi, Csymbols, Ccoef1);

% 
acc_x1 = subs(d2sigma_x, Csymbols, Ccoef1);
acc_y1 = subs(d2sigma_y, Csymbols, Ccoef1);
acc_z1 = subs(d2sigma_z, Csymbols, Ccoef1);

% third traj
Ccoef2 = num2cell(coef(poly_order * 6 + 1 : poly_order * 9));

pos_x2 = subs(sigma_x, Csymbols, Ccoef2);
pos_y2 = subs(sigma_y, Csymbols, Ccoef2);
pos_z2 = subs(sigma_z, Csymbols, Ccoef2);

vel_x2 = subs(dsigma_x, Csymbols, Ccoef2);
vel_y2 = subs(dsigma_y, Csymbols, Ccoef2);
vel_z2 = subs(dsigma_z, Csymbols, Ccoef2);
vel_psi2 = subs(dsigma_psi, Csymbols, Ccoef2);

% 
acc_x2 = subs(d2sigma_x, Csymbols, Ccoef2);
acc_y2 = subs(d2sigma_y, Csymbols, Ccoef2);
acc_z2 = subs(d2sigma_z, Csymbols, Ccoef2);

global T;

%% 1st traj constraints
ceq(1) = double(subs(pos_x, t, 0)) - wayp_x(1);
ceq(2) = double(subs(pos_y, t, 0)) - wayp_y(1);
ceq(3) = double(subs(pos_z, t, 0)) - 0;

ceq(4) = double(subs(vel_x, t, 0)) - 0;
ceq(5) = double(subs(vel_y, t, 0)) - 0;
ceq(6) = double(subs(vel_z, t, 0)) - 0;

ceq(7) = double(subs(pos_x, t, T/2)) - wayp_x(2);
ceq(8) = double(subs(pos_y, t, T/2)) - wayp_y(2);
ceq(9) = double(subs(pos_z, t, T/2)) - 0;

ceq(10) = double(subs(pos_x, t, T)) - wayp_x(3);
ceq(11) = double(subs(pos_y, t, T)) - wayp_y(3);
ceq(12) = double(subs(pos_z, t, T)) - 0;
% 
%% 2nd traj constraints
ceq(13) = double(subs(pos_x1, t, 0)) - wayp_x(3);
ceq(14) = double(subs(pos_y1, t, 0))- wayp_y(3);
ceq(15) = double(subs(pos_z1, t, 0)) - 0;

ceq(16) = double(subs(pos_x1, t, T/2)) - wayp_x(4);
ceq(17) = double(subs(pos_y1, t, T/2)) - wayp_y(4);
ceq(18) = double(subs(pos_z1, t, T/2)) - 0;

ceq(19) = double(subs(pos_x1, t, T)) - wayp_x(5);
ceq(20) = double(subs(pos_y1, t, T)) - wayp_y(5);
ceq(21) = double(subs(pos_z1, t, T)) - 0;

%% 3rd traj constraints
ceq(22) = double(subs(pos_x2, t, 0)) - wayp_x(5);
ceq(23) = double(subs(pos_y2, t, 0))- wayp_y(5);
ceq(24) = double(subs(pos_z2, t, 0)) - 0;

ceq(25) = double(subs(pos_x2, t, T/2)) - wayp_x(6);
ceq(26) = double(subs(pos_y2, t, T/2)) - wayp_y(6);
ceq(27) = double(subs(pos_z2, t, T/2)) - 0;

ceq(28) = double(subs(pos_x2, t, T)) - wayp_x(1);
ceq(29) = double(subs(pos_y2, t, T)) - wayp_y(1);
ceq(30) = double(subs(pos_z2, t, T)) - 0;

ceq(31) = double(subs(vel_x2, t, T)) - 0;
ceq(32) = double(subs(vel_y2, t, T)) - 0;
ceq(33) = double(subs(vel_z2, t, T)) - 0;

%% smooth constraint
ceq(34) = double(subs(vel_x, t, T)) - double(subs(vel_x1, t, 0));
ceq(35) = double(subs(vel_y, t, T)) - double(subs(vel_y1, t, 0));
ceq(36) = double(subs(vel_z, t, T)) - double(subs(vel_z1, t, 0));

ceq(37) = double(subs(acc_x, t, T)) - double(subs(acc_x1, t, 0));
ceq(38) = double(subs(acc_y, t, T)) - double(subs(acc_y1, t, 0));
ceq(39) = double(subs(acc_z, t, T)) - double(subs(acc_z1, t, 0));

ceq(40) = double(subs(vel_x1, t, T)) - double(subs(vel_x2, t, 0));
ceq(41) = double(subs(vel_y1, t, T)) - double(subs(vel_y2, t, 0));
ceq(42) = double(subs(vel_z1, t, T)) - double(subs(vel_z2, t, 0));

ceq(43) = double(subs(acc_x1, t, T)) - double(subs(acc_x2, t, 0));
ceq(44) = double(subs(acc_y1, t, T)) - double(subs(acc_y2, t, 0));
ceq(45) = double(subs(acc_z1, t, T)) - double(subs(acc_z2, t, 0));

%% heading changing rate constraint
% sample_num = 20;
% time_sample = 0: T/sample_num : T;
% vel_psi_max = 3 * pi;
% 
% vel_psi_on_traj = double(subs(vel_psi, t, time_sample));
% vel_psi_on_traj1 = double(subs(vel_psi1, t, time_sample));
% vel_psi_on_traj2 = double(subs(vel_psi2, t, time_sample));
% 
% r = abs(vel_psi_on_traj) - vel_psi_max;
% r(r < 0) = 0;
% ceq(46) = sum(r);
% 
% r1 = abs(vel_psi_on_traj1) - vel_psi_max;
% r1(r1 < 0) = 0;
% ceq(47) = sum(r1);
% 
% r2 = abs(vel_psi_on_traj2) - vel_psi_max ;
% r2(r2 < 0) = 0;
% ceq(48) = sum(r2);
end


function f = objfun(coef)
global parameterValuesAtEachIteration;
parameterValuesAtEachIteration = [parameterValuesAtEachIteration; coef];
global intSumSquare;
global cx;
global cy;
global cz;
global poly_order;

Csymbols = num2cell([cx,cy,cz]);

Ccoef = num2cell(coef(1 : poly_order * 3));
Ccoef1 = num2cell(coef(poly_order * 3 + 1 : poly_order * 6));
Ccoef2 = num2cell(coef(poly_order * 6 + 1 : poly_order * 9));

f = double(subs(intSumSquare, Csymbols, Ccoef)) + ...
    double(subs(intSumSquare, Csymbols, Ccoef1)) + ...
     double(subs(intSumSquare, Csymbols, Ccoef2));
end

