clear all;
syms t;
% T = 2;
load('ball_obstraction.mat')





pos_x_res(t)   = poly2sym(coef(1:7), t);
pos_y_res(t)   = poly2sym(coef(8:14), t);
pos_z_res(t)   = poly2sym(coef(15:21), t);
% psi_res(t)   = poly2sym(coef(22:28), t);

vel_x_res = diff(pos_x_res,t,1);
vel_y_res = diff(pos_y_res,t,1);
vel_z_res = diff(pos_z_res,t,1);

pos_psi_res(t) = atan2(vel_y_res(t),vel_x_res(t));
vel_psi_res(t) = diff(pos_psi_res,t,1);

time_secs = 0 : 0.01 : T;
x_pos = double(pos_x_res(time_secs));
y_pos = double(pos_y_res(time_secs));
z_pos = double(pos_z_res(time_secs));
psi = double(pos_psi_res(time_secs));

x_vel = double(vel_x_res(time_secs));
y_vel = double(vel_y_res(time_secs));
z_vel = double(vel_z_res(time_secs));
psi_vel = double(vel_psi_res(time_secs));

gray = '#a1a3a6';
figure;
% subplot(1,3,1);
set(gcf, 'Position', [100 100 500 500]); 



plot3( x_pos, y_pos, z_pos,'linewidth',2,'color',gray);
axis equal;
% set(gca,'xlim',[-0.1,1.1],'xtick',[-0.1:0.2:1.1]);
hold on;

radius = 0.5;

% Step 2: Generate x, y, and z coordinates for a sphere using 'sphere' function
[x, y, z] = sphere(100);

% Step 3: Scale the coordinates by the radius to get desired size
x = radius + x * radius;
y = radius + y * radius;
z = radius + z * radius;

% Step 4: Plot the ball as a surface plot using 'surf'
ballcolor = '#9b95c9';
h = surf(x, y, z, 'FaceColor',ballcolor, 'EdgeColor', 'none');
alpha(h, 0.5);
light('Position', [10 10 10], 'Style', 'infinite');
lighting gouraud;
shading interp;
set(h, 'FaceColor', ballcolor, 'AmbientStrength', 0.6, 'DiffuseStrength', 0.5, 'SpecularStrength', 0.2, 'SpecularExponent', 20);


showpoint = 11;

downsamples = 2 : floor(size(time_secs,2)/showpoint) : (size(time_secs,2)-1);
for downsample = downsamples
    drawframe( [x_pos(downsample),y_pos(downsample), z_pos(downsample)]',rotz(rad2deg(psi(downsample))),0.2)
end
grid on;

red_color = '#a7324a';
green_color = '#2b6447';
blue_color = '#145b7d';
purple_color = '#472d56';
filler = '#f6f5ec';

curve_width = 1.2;

figure;
subplot(4,1,1); 
plot(time_secs, x_pos, 'Linewidth', curve_width, 'color', red_color);
hold on;
scatter(time_secs(downsamples), x_pos(downsamples), 'MarkerEdgeColor', red_color, 'MarkerFaceColor',filler);

subplot(4,1,2);
plot(time_secs, y_pos, 'Linewidth', curve_width, 'color', green_color);
hold on;
scatter(time_secs(downsamples), y_pos(downsamples), 'MarkerEdgeColor', green_color, 'MarkerFaceColor',filler);

subplot(4,1,3);
plot(time_secs, z_pos, 'Linewidth', curve_width, 'color', blue_color);
hold on;
scatter(time_secs(downsamples), z_pos(downsamples), 'MarkerEdgeColor', blue_color, 'MarkerFaceColor',filler);

subplot(4,1,4);
plot(time_secs(2:end-1), psi(2:end-1), 'Linewidth', curve_width, 'color', purple_color);
hold on;
scatter(time_secs(downsamples), psi(downsamples), 'MarkerEdgeColor', purple_color, 'MarkerFaceColor',filler);

set(gcf, 'Position', [100 100 500 450]); 

figure;
subplot(4,1,1); 
plot(time_secs, x_vel, 'Linewidth', curve_width, 'color', red_color);
hold on;
scatter(time_secs(downsamples), x_vel(downsamples), 'MarkerEdgeColor', red_color, 'MarkerFaceColor',filler);

subplot(4,1,2);
plot(time_secs, y_vel, 'Linewidth', curve_width, 'color', green_color);
hold on;
scatter(time_secs(downsamples), y_vel(downsamples), 'MarkerEdgeColor', green_color, 'MarkerFaceColor',filler);

subplot(4,1,3);
plot(time_secs, z_vel, 'Linewidth', curve_width, 'color', blue_color);
hold on;
scatter(time_secs(downsamples), z_vel(downsamples), 'MarkerEdgeColor', blue_color, 'MarkerFaceColor',filler);

subplot(4,1,4);
plot(time_secs(2:end-1), psi_vel(2:end-1), 'Linewidth', curve_width, 'color', purple_color);
hold on;
scatter(time_secs(downsamples), psi_vel(downsamples), 'MarkerEdgeColor', purple_color, 'MarkerFaceColor',filler);

set(gcf, 'Position', [100 100 500 450]);