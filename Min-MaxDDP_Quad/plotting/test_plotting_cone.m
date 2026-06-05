%% %---------------- 
% plot frame vectors
clear;

cone.base_position = [0 0 0];
cone.base_radius = 0.1;
cone.top_radius = 0;

offset = [4 4 1];
length = [1 1 1];
body_ratio = [0.8 0.8 0.8];
 
figure(1);
clf;


position = [1 1 3];
orientation_body_wrt_inertia = [0 0 0 0]/180*pi;

in_args.cone = cone;
in_args.Npoints = 10;
in_args.offset = offset;
in_args.length =length;
in_args.body_ratio = body_ratio;

plot_frame (position, orientation_body_wrt_inertia,in_args)

axis equal;   
view(3);
grid on;

box on;

xlim([-5 5]);
ylim([-5 5]);
zlim([-5 5]);

%%
figure(2);
clf;

hold on;

position = [2 0.5 -1];
orientation_body_wrt_inertia = [10 30 20]/180*pi;

plot_quadrotor_new(position, orientation_body_wrt_inertia);

axis equal;   
%view([1 1 1]);
view(3);
grid on;

box on;

xlim([-3 3]);
ylim([-3 3]);
zlim([-3 3]);

xlabel('X');
ylabel('Y');
zlabel('Z');