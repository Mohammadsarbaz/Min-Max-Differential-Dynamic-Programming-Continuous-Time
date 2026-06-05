clear;

N = 25;
theta = linspace(0,2*pi,N);

pos_x = 0;
pos_y = 0;
pos_z = 0;

r = 4;
h = 3;

figure(1);
clf;

view(3);
point_list_up = zeros(N,3);
point_list_down = zeros(N,3);

for i = 1:N
   point_list (1) = pos_x + r * cos(theta(i));
   point_list (2) = pos_y + r * sin(theta(i));

   point_list_up (i,:) = [point_list, pos_z + h/2];
   point_list_down (i,:) = [point_list, pos_z - h/2];
end

hold on;
plot3(point_list_up(:,1), point_list_up(:,2), point_list_up(:,3));
plot3(point_list_down(:,1), point_list_down(:,2), point_list_down(:,3));


vertex_list = [point_list_up; point_list_down];
face_list = zeros(N-1,4);

for k = 1:N-1
    face_list(k,:) = [k k+1 k + N+1 k+N]; 
end

p = patch('Vertices', vertex_list, 'Faces', face_list,'FaceColor','red');

p = patch('Vertices', point_list_up, 'Faces', [1:N],'FaceColor','blue');

p = patch('Vertices', point_list_down, 'Faces', [1:N],'FaceColor','yellow');

grid on;
axis equal;
