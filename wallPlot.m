clear all;
syms t;
T = 2;
load('wall_obstraction.mat')





pos_x_res(t)   = poly2sym(coef(1:7), t);
pos_y_res(t)   = poly2sym(coef(8:14), t);
pos_z_res(t)   = poly2sym(coef(15:21), t);

pos_x_res1(t)   = poly2sym(coef(22:28), t);
pos_y_res1(t)   = poly2sym(coef(29:35), t);
pos_z_res1(t)   = poly2sym(coef(36:42), t);

% psi_res(t)   = poly2sym(coef(22:28), t);

vel_x_res = diff(pos_x_res,t,1);
vel_y_res = diff(pos_y_res,t,1);
vel_z_res = diff(pos_z_res,t,1);

vel_x_res1 = diff(pos_x_res1,t,1);
vel_y_res1 = diff(pos_y_res1,t,1);
vel_z_res1 = diff(pos_z_res1,t,1);

pos_psi_res(t) = atan2(vel_y_res(t),vel_x_res(t));
pos_psi_res1(t) = atan2(vel_y_res1(t),vel_x_res1(t));

vel_psi_res(t) = diff(pos_psi_res,t,1);
vel_psi_res1(t) = diff(pos_psi_res1,t,1);

time_secs = 0.01 : 0.01 : T;
x_pos = double(pos_x_res(time_secs));
y_pos = double(pos_y_res(time_secs));
z_pos = double(pos_z_res(time_secs));
psi = double(pos_psi_res(time_secs));

x_vel = double(vel_x_res(time_secs));
y_vel = double(vel_y_res(time_secs));
z_vel = double(vel_z_res(time_secs));
psi_vel = double(vel_psi_res(time_secs));

x_pos1 = double(pos_x_res1(time_secs));
y_pos1 = double(pos_y_res1(time_secs));
z_pos1 = double(pos_z_res1(time_secs));
psi1 = double(pos_psi_res1(time_secs));

x_vel1 = double(vel_x_res1(time_secs));
y_vel1 = double(vel_y_res1(time_secs));
z_vel1 = double(vel_z_res1(time_secs));
psi_vel1 = double(vel_psi_res1(time_secs));

gray = '#a1a3a6';
light = '#f7acbc';

figure;
% subplot(1,3,1);
set(gcf, 'Position', [100 100 500 500]); 

hold on;

plot3( x_pos, y_pos, z_pos,'linewidth',2,'color',gray);
hold on;
plot3( x_pos1, y_pos1, z_pos1,'linewidth',2,'color',light);
axis equal;
% set(gca,'xlim',[-0.1,1.1],'xtick',[-0.1:0.2:1.1]);

%% cylinder
% Define parameters of the cylinder
radius = 1;    % Radius of the base
height = 1;   % Height of the cylinder

% Create coordinate arrays for x and y axes
theta = linspace(0, 2*pi, 100);   % Angles around the circumference
z = linspace(0, height, 50);       % Heights along the axis

[THETA,Z] = meshgrid(theta,z);

% Generate coordinates for X, Y and Z using cylindrical coordinates equation
X = radius * cos(THETA);
Y = radius * sin(THETA);

% Create a figure and axes

ax = gca;

% Draw a transparent cylinder using 'surf' or 'mesh'

ballcolor = '#9b95c9';
x = 0.3 .* X + 0.5;
y = 0.3 .* Y - 0.2;
z = Z -0.5;
h = surf(z,x,y, 'FaceColor',ballcolor, 'EdgeColor', 'none');
% alpha(h, 0.5);
% light('Position', [10 10 10], 'Style', 'infinite');
% lightangle(-45,30)


x1 = 0.3 .* X + 1.5;
y1 = 0.3 .* Y + 0.1;
z1 = Z -0.5;
h1 = surf(z1,x1,y1, 'FaceColor',ballcolor, 'EdgeColor', 'none');
alpha(h1, 0.5);
lightangle(-45,30)
shading interp;

set(h, 'FaceColor', ballcolor, 'AmbientStrength', 0.6, 'DiffuseStrength', 0.5, 'SpecularStrength', 0.2, 'SpecularExponent', 20);

set(h1, 'FaceColor', ballcolor, 'AmbientStrength', 0.6, 'DiffuseStrength', 0.5, 'SpecularStrength', 0.2, 'SpecularExponent', 20);

% view([1,1,1])
%%

showpoint = 6;
axislength = 0.2;


downsamples = 1 : floor(size(time_secs,2)/showpoint) : (size(time_secs,2));
for downsample = downsamples
    drawframe( [x_pos(downsample),y_pos(downsample), z_pos(downsample)]',rotz(rad2deg(psi(downsample))),axislength)
end
downsamples_1 = floor(size(time_secs,2)/showpoint) : floor(size(time_secs,2)/showpoint) : (size(time_secs,2)-1);
for downsample = downsamples_1
    drawframe( [x_pos1(downsample),y_pos1(downsample), z_pos1(downsample)]',rotz(rad2deg(psi1(downsample))),axislength)
end
grid on;

red_color = '#a7324a'; red_color1 = '#d2553d';
green_color = '#2b6447'; green_color1 = '#375830';
blue_color = '#145b7d'; blue_color1 = '#008792';
purple_color = '#472d56'; purple_color1 = '#472d56';
filler = '#f6f5ec';

curve_width = 1.2;

figure;
subplot(4,1,1); 
hold on;
plot(time_secs, x_pos, 'Linewidth', curve_width, 'color', red_color);
plot(T + time_secs, x_pos1, '--','Linewidth', curve_width, 'color', red_color);
ylim([-0.5,0.5]);

scatter(time_secs(downsamples), x_pos(downsamples), 'MarkerEdgeColor', red_color, 'MarkerFaceColor',filler);
scatter(T + time_secs(downsamples_1), x_pos1(downsamples_1), 'MarkerEdgeColor', red_color, 'MarkerFaceColor',filler);

subplot(4,1,2);
hold on;
plot(time_secs, y_pos, 'Linewidth', curve_width, 'color', green_color);
plot(T + time_secs, y_pos1,'--', 'Linewidth', curve_width, 'color', green_color);

scatter(time_secs(downsamples), y_pos(downsamples), 'MarkerEdgeColor', green_color, 'MarkerFaceColor',filler);
scatter(T + time_secs(downsamples_1), y_pos1(downsamples_1), 'MarkerEdgeColor', green_color, 'MarkerFaceColor',filler);

subplot(4,1,3);
hold on;
plot(time_secs, z_pos, 'Linewidth', curve_width, 'color', blue_color);
plot(T + time_secs, z_pos1,'--', 'Linewidth', curve_width, 'color', blue_color);

scatter(time_secs(downsamples), z_pos(downsamples), 'MarkerEdgeColor', blue_color, 'MarkerFaceColor',filler);
scatter(T + time_secs(downsamples_1), z_pos1(downsamples_1), 'MarkerEdgeColor', blue_color, 'MarkerFaceColor',filler);

subplot(4,1,4);
hold on;
plot(time_secs(2:end-1), psi(2:end-1), 'Linewidth', curve_width, 'color', purple_color);
plot(T + time_secs(2:end-1), psi1(2:end-1),'--', 'Linewidth', curve_width, 'color', purple_color);
ylim([-pi/4 + pi /2,pi/4 + pi/2]);

scatter(time_secs(downsamples), psi(downsamples), 'MarkerEdgeColor', purple_color, 'MarkerFaceColor',filler);
scatter(T + time_secs(downsamples_1), psi1(downsamples_1), 'MarkerEdgeColor', purple_color, 'MarkerFaceColor',filler);

set(gcf, 'Position', [100 100 500 450]); 

figure;
subplot(4,1,1); 
hold on;
plot(time_secs, x_vel, 'Linewidth', curve_width, 'color', red_color);
plot(T + time_secs, x_vel1,'--', 'Linewidth', curve_width, 'color', red_color);

scatter(time_secs(downsamples), x_vel(downsamples), 'MarkerEdgeColor', red_color, 'MarkerFaceColor',filler);
scatter(T + time_secs(downsamples_1), x_vel1(downsamples_1), 'MarkerEdgeColor', red_color, 'MarkerFaceColor',filler);
ylim([-0.5,0.5]);

subplot(4,1,2);
hold on;
plot(time_secs, y_vel, 'Linewidth', curve_width, 'color', green_color);
plot(T + time_secs, y_vel1,'--', 'Linewidth', curve_width, 'color', green_color);

scatter(time_secs(downsamples), y_vel(downsamples), 'MarkerEdgeColor', green_color, 'MarkerFaceColor',filler);
scatter(T + time_secs(downsamples_1), y_vel1(downsamples_1), 'MarkerEdgeColor', green_color, 'MarkerFaceColor',filler);

subplot(4,1,3);
hold on;
plot(time_secs, z_vel, 'Linewidth', curve_width, 'color', blue_color);
plot(T + time_secs, z_vel1,'--', 'Linewidth', curve_width, 'color', blue_color);

scatter(time_secs(downsamples), z_vel(downsamples), 'MarkerEdgeColor', blue_color, 'MarkerFaceColor',filler);
scatter(T + time_secs(downsamples_1), z_vel1(downsamples_1), 'MarkerEdgeColor', blue_color, 'MarkerFaceColor',filler);

subplot(4,1,4);
hold on;
plot(time_secs(2:end-1), psi_vel(2:end-1), 'Linewidth', curve_width, 'color', purple_color);
plot(T + time_secs(2:end-1), psi_vel1(2:end-1),'--', 'Linewidth', curve_width, 'color', purple_color);

scatter(time_secs(downsamples), psi_vel(downsamples), 'MarkerEdgeColor', purple_color, 'MarkerFaceColor',filler);
scatter(T + time_secs(downsamples_1), psi_vel1(downsamples_1), 'MarkerEdgeColor', purple_color, 'MarkerFaceColor',filler);

ylim([-pi/4 ,pi/4]);

set(gcf, 'Position', [100 100 500 450]);