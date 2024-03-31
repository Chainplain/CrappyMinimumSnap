clear all;

poly_order = 7;
c0 = rand(1, poly_order * 3);
filename = "ball_obstraction";

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
global sigma_psi;

sigma_x = poly2sym(cx, t);
sigma_y = poly2sym(cy, t);
sigma_z = poly2sym(cz, t);
% sigma_psi = poly2sym(cpsi, t);


mu_p = 1;
mu_psi = 1;
mu_2 = 1;

global T;
T = 3;

global start_x;
global start_y;
global start_z;


start_x = 0;
start_y = 0;
start_z = 0;


global termination_x;
global termination_y;
global termination_z;


termination_x = 1;
termination_y = 1;
termination_z = 1;


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
global dsigma_psi;

dsigma_x = diff(sigma_x,t,1);
dsigma_y = diff(sigma_y,t,1);
dsigma_z = diff(sigma_z,t,1);
sigma_psi = atan2(dsigma_y,dsigma_x );
dsigma_psi = diff(sigma_psi,t,1);

d4sigma_x(t) = diff(sigma_x,t,4);
d4sigma_y(t) = diff(sigma_y,t,4);
d4sigma_z(t) = diff(sigma_z,t,4);
% d4sigma_psi(t) = diff(sigma_psi,t,4);

psi_rat_cons = (0.5 * pi)^2 - dsigma_psi^2;
% rec_psi_rat_cons = (1 + sign(psi_rat_cons)) * psi_rat_cons;

sumSquare(t) = mu_p * d4sigma_x(t) ^ 2 + mu_2 * dsigma_x ^ 2 +...
               mu_p * d4sigma_y(t) ^ 2 + mu_2 * dsigma_y ^ 2 +...
               mu_p * d4sigma_z(t) ^ 2 + mu_2 * dsigma_z ^ 2 + psi_rat_cons ;
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
% global sigma_psi;
% 
global dsigma_x;
global dsigma_y;
global dsigma_z;
global dsigma_psi;
%
global cx;
global cy;
global cz;
% global cpsi;
% 
Csymbols = num2cell([cx,cy,cz]);
Ccoef = num2cell(coef);
% 
pos_x = subs(sigma_x, Csymbols, Ccoef);
pos_y = subs(sigma_y, Csymbols, Ccoef);
pos_z = subs(sigma_z, Csymbols, Ccoef);
% pos_psi = subs(sigma_psi, Csymbols, Ccoef);

vel_x = subs(dsigma_x, Csymbols, Ccoef);
vel_y = subs(dsigma_y, Csymbols, Ccoef);
vel_z = subs(dsigma_z, Csymbols, Ccoef);
vel_psi = subs(dsigma_psi, Csymbols, Ccoef);
% ceq = [];
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
% ceq(16) = double(subs(vel_psi, t, 0)) - 0;

% 
%% termination position constraints
global termination_x;
global termination_y;
global termination_z;

global termination_vx;
global termination_vy;
global termination_vz;

global T;

ceq(7) = double(subs(pos_x, t, T))- termination_x;
ceq(8) = double(subs(pos_y, t, T)) - termination_y;
ceq(9) = double(subs(pos_z, t, T)) - termination_z;

ceq(10) = double(subs(vel_x, t, T)) - termination_vx;
ceq(11) = double(subs(vel_y, t, T)) - termination_vy;
ceq(12) = double(subs(vel_z, t, T)) - termination_vz;
% ceq(17) = double(subs(vel_psi, t, T)) - 0;

%% obstacle avoidance constraints
sample_num = 20;
time_sample = 0: T/sample_num : T;

pos_x_on_traj = double(subs(pos_x, t, time_sample));
pos_y_on_traj = double(subs(pos_y, t, time_sample));
pos_z_on_traj = double(subs(pos_z, t, time_sample));


r = 0.5 - sqrt((pos_x_on_traj - 0.5).^2 + (pos_y_on_traj - 0.5).^2 + (pos_z_on_traj - 0.5).^2);
r(r < 0) = 0;
ceq(13) = sum(r);

% ceq(9) = double(subs(pos_x, t, T))- 0.5;
%% velocity constraints
global max_v_lateral;
global max_v_vertical;

vel_x_on_traj = double(subs(vel_x, t, time_sample));
vel_y_on_traj = double(subs(vel_y, t, time_sample));
vel_z_on_traj = double(subs(vel_z, t, time_sample));

velL =  sqrt((vel_x_on_traj).^2 + (vel_y_on_traj).^2) - max_v_lateral;
velL(velL < 0) = 0;
ceq(14) = sum(velL);

velH = abs(vel_z_on_traj) - max_v_vertical;
velH(velH < 0) = 0;
ceq(15) = sum(velH);

%% heading changing rate constraint
% sample_num = 20;
% time_sample = T/sample_num: T/sample_num : T-T/sample_num;
% vel_psi_max = 0.5 * pi;
% 
% vel_psi_on_traj = double(subs(vel_psi, t, time_sample));
% r = abs(vel_psi_on_traj) - vel_psi_max;
% r(r < 0) = 0;
% ceq(16) = sum(r);
end


function f = objfun(coef)
global parameterValuesAtEachIteration;
parameterValuesAtEachIteration = [parameterValuesAtEachIteration; coef];
global intSumSquare;
global cx;
global cy;
global cz;
global cpsi;

Csymbols = num2cell([cx,cy,cz,cpsi]);
Ccoef = num2cell(coef);

f = double(subs(intSumSquare, Csymbols, Ccoef));
end

