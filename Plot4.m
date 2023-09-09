syms t;
% T = 2;


pos_x_res(t)   = poly2sym(coef(1:7), t);
pos_y_res(t)   = poly2sym(coef(8:14), t);
pos_z_res(t)   = poly2sym(coef(15:21), t);
psi_res(t)   = poly2sym(coef(22:28), t);

vel_x_res = diff(pos_x_res,t,1);
vel_y_res = diff(pos_y_res,t,1);

% pos_psi_res(t) = atan2(vel_y_res(t),vel_x_res(t));

time_secs = 0 : 0.01 : T;
x_pos = double(pos_x_res(time_secs));
y_pos = double(pos_y_res(time_secs));
z_pos = double(pos_z_res(time_secs));
psi = double(psi_res(time_secs));

gray = '#a1a3a6';
figure;
plot3( x_pos, y_pos, z_pos,'linewidth',2,'color',gray);
axis equal;
hold on;

showpoint = 11;
for downsample = 2 : floor(size(time_secs,2)/showpoint) : (size(time_secs,2)-1)
    drawframe( [x_pos(downsample),y_pos(downsample), z_pos(downsample)]',rotz((psi(downsample))),0.2)
end
grid on;