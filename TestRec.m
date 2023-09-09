	
% specifies all the vertices that comprises the object you want to draw

vert = [-0.5 -0.5 -0.5;  ...

        -0.5 0.5 -0.5;  ...

         0.5 0.5 -0.5;  ...

         0.5 -0.5 -0.5; ...

        -0.5 -0.5 0.5; ...

         -0.5 0.5 0.5;  ...

          0.5 0.5 0.5; ...

          0.5 -0.5 0.5];

 

% define the arbitrary polygon(patch) using the vertice number(index) you defined above.

fac = [1 2 3 4; ...

    2 6 7 3; ...

    4 3 7 8; ...

    1 5 8 4; ...

    1 2 6 5; ...

    5 6 7 8];

 

% specify patch (polygons) in patch() function

% just call the patch function multiple times to draw multiple cubes

patch('Faces',fac,'Vertices',vert,'FaceColor','r');  % draw the red cube
hold on;

patch('Faces',fac,'Vertices',(0.5+1*vert),'FaceColor','b');  % draw the blue cube

axis([-2 2 -2 2 -2 2]);

grid();

% material shiny;
% 
% alpha('color');
% 
% alphamap('rampdown');
% 
% view(30,30);