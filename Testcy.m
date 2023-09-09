% Define parameters of the cylinder
radius = 1;    % Radius of the base
height = 2;   % Height of the cylinder

% Create coordinate arrays for x and y axes
theta = linspace(0, 2*pi, 100);   % Angles around the circumference
z = linspace(0, height, 50);       % Heights along the axis

[THETA,Z] = meshgrid(theta,z);

% Generate coordinates for X, Y and Z using cylindrical coordinates equation
X = radius * cos(THETA);
Y = radius * sin(THETA);

% Create a figure and axes
figure;
ax = gca;

% Draw a transparent cylinder using 'surf' or 'mesh'
h = surf(ax,Y,Z,X,'FaceColor','blue');
alpha(h,.5); % Set transparency level to 0.5 (adjust as needed)

axis equal;