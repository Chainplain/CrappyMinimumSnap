clear all;
global poly_order
poly_order = 7;
c0 = rand(1, poly_order * 6);
filename = "wall_obstraction";

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
% global sigma_psi;

sigma_x = poly2sym(cx, t);
sigma_y = poly2sym(cy, t);
sigma_z = poly2sym(cz, t);
% sigma_psi = poly2sym(cpsi, t);

mu_p = 1;
mu_psi = 1;


global T;
T = 2;

global start_x;
global start_y;
global start_z;


start_x = 0;
start_y = 0;
start_z = 0;


global termination_x;
global termination_y;
global termination_z;


termination_x = 0;
termination_y = 2;
termination_z = 0;


global start_vx;
global start_vy;
global start_vz;


start_vx = 0;
start_vy = 0;
start_vz = 0;


global termination_vx;
global termination_vy;
global termination_vz;


termination_vx = 0;
termination_vy = 0;
termination_vz = 0;


global max_v_lateral;
global max_v_vertical;
max_v_lateral  = 0.8;
max_v_vertical = 0.5;


global dsigma_x;
global dsigma_y;
global dsigma_z;


dsigma_x = diff(sigma_x,t,1);
dsigma_y = diff(sigma_y,t,1);
dsigma_z = diff(sigma_z,t,1);
% sigma_psi = atan2(dsigma_x, dsigma_y);
% dsigma_psi = diff(sigma_psi,t,1);

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

sumSquare(t) = mu_p * d4sigma_x(t) ^ 2 + mu_p * dsigma_x ^ 2 + ...
               mu_p * d4sigma_y(t) ^ 2 + mu_p * dsigma_y ^ 2 +...
               mu_p * d4sigma_z(t) ^ 2 + mu_p * dsigma_z ^ 2;
%                mu_psi * d4sigma_psi(t) ^ 2;

global intSumSquare;
intSumSquare = int (sumSquare(t), t, 0, T);

[coef,fval] = fmincon(@objfun,c0,A,b,Aeq,beq,lb,ub,@confuneq,options);

save (filename + ".mat","coef","T")



function [c,ceq] = confuneq(coef)
global t
% Nonlinear inequality constraints
c = [];

% Nonlinear equality constraints
global sigma_x;
global sigma_y;
global sigma_z;

% 
global dsigma_x;
global dsigma_y;
global dsigma_z;

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
% 
acc_x = subs(d2sigma_x, Csymbols, Ccoef);
acc_y = subs(d2sigma_y, Csymbols, Ccoef);
acc_z = subs(d2sigma_z, Csymbols, Ccoef);
% second traj
Ccoef1 = num2cell(coef(poly_order * 3 + 1 : end));

pos_x1 = subs(sigma_x, Csymbols, Ccoef1);
pos_y1 = subs(sigma_y, Csymbols, Ccoef1);
pos_z1 = subs(sigma_z, Csymbols, Ccoef1);

vel_x1 = subs(dsigma_x, Csymbols, Ccoef1);
vel_y1 = subs(dsigma_y, Csymbols, Ccoef1);
vel_z1 = subs(dsigma_z, Csymbols, Ccoef1);
% 
acc_x1 = subs(d2sigma_x, Csymbols, Ccoef1);
acc_y1 = subs(d2sigma_y, Csymbols, Ccoef1);
acc_z1 = subs(d2sigma_z, Csymbols, Ccoef1);
%% start position constraints

global start_x;
global start_y;
global start_z;

global start_vx;
global start_vy;
global start_vz;

ceq(1) = double(subs(pos_x, t, 0)) - start_x;
ceq(2) = double(subs(pos_y, t, 0)) - start_y;
ceq(3) = double(subs(pos_z, t, 0)) - start_z;

ceq(4) = double(subs(vel_x, t, 0)) - start_vx;
ceq(5) = double(subs(vel_y, t, 0)) - start_vy;
ceq(6) = double(subs(vel_z, t, 0)) - start_vz;

% 
%% termination position constraints
global termination_x;
global termination_y;
global termination_z;

global termination_vx;
global termination_vy;
global termination_vz;

global T;

ceq(7) = double(subs(pos_x1, t, T))- termination_x;
ceq(8) = double(subs(pos_y1, t, T)) - termination_y;
ceq(9) = double(subs(pos_z1, t, T)) - termination_z;

ceq(10) = double(subs(vel_x1, t, T)) - termination_vx;
ceq(11) = double(subs(vel_y1, t, T)) - termination_vy;
ceq(12) = double(subs(vel_z1, t, T)) - termination_vz;

%% Continuity constraint
ceq(13) = double(subs(pos_x, t, T)) - double(subs(pos_x1, t, 0));
ceq(14) = double(subs(pos_y, t, T)) - double(subs(pos_y1, t, 0));
ceq(15) = double(subs(pos_z, t, T)) - double(subs(pos_z1, t, 0));

ceq(16) = double(subs(vel_x, t, T)) - double(subs(vel_x1, t, 0));
ceq(17) = double(subs(vel_y, t, T)) - double(subs(vel_y1, t, 0));
ceq(18) = double(subs(vel_z, t, T)) - double(subs(vel_z1, t, 0));

ceq(19) = double(subs(acc_x, t, T)) - double(subs(acc_x1, t, 0));
ceq(20) = double(subs(acc_y, t, T)) - double(subs(acc_y1, t, 0));
ceq(21) = double(subs(acc_z, t, T)) - double(subs(acc_z1, t, 0));

%% obstacle avoidance constraints
sample_num = 20;
time_sample = 0: T/sample_num : T;
 
pos_y_on_traj = double(subs(pos_y, t, time_sample));
% pos_y_on_traj = double(subs(pos_y, t, time_sample));
pos_z_on_traj = double(subs(pos_z, t, time_sample));

pos_y_on_traj1 = double(subs(pos_y1, t, time_sample));
pos_z_on_traj1 = double(subs(pos_z1, t, time_sample));
% 
% 

r = 0.3 - sqrt((pos_y_on_traj - 0.5).^2 +  (pos_z_on_traj +0.2).^2);
r(r < 0) = 0;
ceq(22) = sum(r);

r1 = 0.3 - sqrt((pos_y_on_traj - 1.5).^2 +  (pos_z_on_traj - 0.1).^2);
r1(r1 < 0) = 0;
ceq(23) = sum(r1);

r2 = 0.3 - sqrt((pos_y_on_traj1 - 0.5).^2 +  (pos_z_on_traj1 +0.2).^2);
r2(r2 < 0) = 0;
ceq(24) = sum(r2);

r3 = 0.3 - sqrt((pos_y_on_traj1 - 1.5).^2 +  (pos_z_on_traj1 - 0.1).^2);
r3(r3 < 0) = 0;
ceq(25) = sum(r3);

% h1 = pos_z_on_traj - 0.25; 
% h1(h1 < 0) = 0;
% ceq(26) = sum(h1);
% 
% h2 = - pos_z_on_traj + 0.25; 
% h2(h2 < 0) = 0;
% ceq(27) = sum(h2);

% h3 = pos_z_on_traj1 - 0.25; 
% h3(h3 < 0) = 0;
% ceq(28) = sum(h3);

% h4 = - pos_z_on_traj1 + 0.25; 
% h4(h4 < 0) = 0;
% ceq(29) = sum(h4);

% ceq(9) = double(subs(pos_x, t, T))- 0.5;
%% velocity constraints
% global max_v_lateral;
% global max_v_vertical;
% 
% vel_x_on_traj = double(subs(vel_x, t, time_sample));
% vel_y_on_traj = double(subs(vel_y, t, time_sample));
% vel_z_on_traj = double(subs(vel_z, t, time_sample));
% 
% velL =  sqrt((vel_x_on_traj).^2 + (vel_y_on_traj).^2) - max_v_lateral;
% velL(velL < 0) = 0;
% ceq(14) = sum(velL);
% 
% velH = abs(vel_z_on_traj) - max_v_vertical;
% velH(velH < 0) = 0;
% ceq(15) = sum(velH);

%% heading constraints
% relax = 0.1;
% pos_psi_on_traj = (double(subs(pos_psi, t, time_sample)));
% de_theta_on_traj = atan2( vel_y_on_traj, vel_x_on_traj);
% hc = 1 - relax - cos(pos_psi_on_traj) .* cos(de_theta_on_traj) ...
%        - sin(pos_psi_on_traj) .* sin(de_theta_on_traj);
% hc(hc < 0) = 0;   
% ceq(16) = sum(hc);   
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

f = double(subs(intSumSquare, Csymbols, Ccoef)) + ...
    double(subs(intSumSquare, Csymbols, Ccoef1));
end

