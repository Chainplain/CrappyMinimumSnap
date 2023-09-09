% Step 2: Generate x, y, and z coordinates for a sphere using 'sphere' function
[x, y, z] = sphere(100);

% Step 3: Scale the coordinates by the radius to get desired size
x = x * radius;
y = y * radius;
z = z * radius;

% Step 4: Plot the ball as a surface plot using 'surf'
figure;
h = surf(x, y, z, 'EdgeColor', 'none');

% Step 5: Set material properties to make the ball appear red
set(h, 'FaceColor', 'r', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.5, 'SpecularStrength', 0.2, 'SpecularExponent', 20);

% Step 6: Add lighting and shading effects
light('Position', [10 10 10], 'Style', 'infinite');
lighting gouraud;
shading interp;
set(h, 'FaceColor', 'r', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.5, 'SpecularStrength', 0.2, 'SpecularExponent', 20);

% Optional steps for visualization enhancement:
axis equal; % Set aspect ratio to be equal on all axes
axis off;   % Turn off axis lines and labels