


% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% axis equal;
% grid on;
% view([-27 14]);
% 
% 
% cam_x0 = -18;
% cam_y0 = 0;
% cam_z0 = 15;
% 
% roll0 = pi;
% pitch0 = pi/4;
% yaw0 = pi/10;
% 
% len = 5*ones(1,3);
% center = [0 0 0];
% plot_box(center, len);
% plot_camera([cam_x0 cam_y0 cam_z0], ...
%             [roll0 pitch0 yaw0], [2 2], 5);
% 
% 
% order = 'rpy';
%     
% Rwc = dcmfromeuler(roll0,pitch0,yaw0,order);
% 
% lx = len(1);
% ly = len(2);
% lz = len(3);
% cx = center(1);
% cy = center(2);
% cz = center(3);
% 
% 
% verts = ones(8,1)*[cx cy cz] + ([0 0 0;
%                                  1 0 0; 
%                                  1 1 0;
%                                  0 1 0;
%                                  0 1 1;
%                                  1 1 1;
%                                  1 0 1;
%                                  0 0 1]-0.5*ones(8,3)).*(ones(8,1)*[lx ly lz]);
% 
% verts_c = zeros(8,3);                             
% for i =1:8 
%     verts_c(i,:) = (verts(i,:) - [cam_x0 cam_y0 cam_z0])*Rwc;
% end
% 
% verts_im = [verts_c(:,1)./verts_c(:,3) verts_c(:,2)./verts_c(:,3)];
% 
%    faces = [ 1 2 3 4; ...
%               1 2 7 8; ...
%               1 4 5 8; ...
%               2 3 6 7; ... 
%               3 4 5 6; ...
%               5 6 7 8];
% 
%     cdata(:,:,1) = [1 0 0 1 1 0 ];
%     cdata(:,:,2) = [0 0 0 0 1 1 ];
%     cdata(:,:,3) = [1 1 0 0 0 0 ];
% 
%     fvcdata(:,:,1) = [0 0 0 0 1 1 1 1];
%     fvcdata(:,:,2) = [0 0 1 1 0 0 1 1];
%     fvcdata(:,:,3) = [0 1 0 1 0 1 0 1];
% 
%     fvcdata = [0 0 1;
%                0 0 0;
%                1 0 0;
%                1 1 0;
%                0 1 0;
%                0 1 1;
%                1 1 1;
%                1 0 1];
%            
%        cdata = [1 0 1;
%                0 0 1;
%                0 0 0;
%                1 0 0;
%                1 1 0;
%                0 1 0]; 
%            
%    figure
%     p = patch('Vertices',verts_im, ...
%               'Faces',faces, ...
%               'FaceColor', 'flat', ...
%               'FaceVertexCData', cdata);
%     hold on;      
%     scatter(verts_im(:,1), verts_im(:,2),...
%           'o', 'filled', ...
%           'CData', fvcdata);
%           grid on;
%           axis equal;
          
          %%
%%% initial

% num_points = 3;
% cam_x0 = 0;
% cam_y0 = -15;
% cam_z0 = 20;
% 
% roll0 = pi/4;
% pitch0 = pi;
% yaw0 = pi/10;
% 
% cam_pose0_w = [cam_x0 cam_y0 cam_z0];
% cam_orientation0_w = [roll0 pitch0 yaw0 ];
% 
% len = 5*ones(1,3);
% center = [0 0 0];
% 
% lx = len(1);
% ly = len(2);
% lz = len(3);
% cx = center(1);
% cy = center(2);
% cz = center(3);
% 
% 
% f_pts_pose_w = ones(8,1)*[cx cy cz] + ([0 0 0;
%                                  1 0 0; 
%                                  1 1 0;
%                                  0 1 0;
%                                  0 1 1;
%                                  1 1 1;
%                                  1 0 1;
%                                  0 0 1]-0.5*ones(8,3)).*(ones(8,1)*[lx ly lz]);
%                              
% f_pts_pose0_c = transform2camera_frame (cam_pose0_w, cam_orientation0_w, f_pts_pose_w);
% 
% S0 = zeros(3*num_points+3,1);
% 
% 
% for i = 1:num_points
%     pX = f_pts_pose0_c(i,1);
%     pY = f_pts_pose0_c(i,2);
%     pZ = f_pts_pose0_c(i,3);
%     
%     S0(2*i-1:2*i) =  [pX/pZ pY/pZ]'; 
%     S0(2*num_points+i) = pZ;
% end
% 
% S0(3*num_points+1) = roll0;
% S0(3*num_points+2) = pitch0;
% S0(3*num_points+3) = yaw0;
% 
% 
% %%% desired
% cam_xd = -18;
% cam_yd = 0;
% cam_zd = 15;
% 
% rolld = pi;
% pitchd = pi/4;
% yawd = pi/10;
% 
% cam_posed_w = [cam_xd cam_yd cam_zd];
% cam_orientationd_w = [rolld pitchd yawd ];
%                             
% f_pts_posed_c = transform2camera_frame (cam_posed_w, cam_orientationd_w, f_pts_pose_w);
% 
% 
% global Sd;
% Sd = zeros(2*num_points,1);
% 
% for i = 1:num_points
%     pX = f_pts_posed_c(i,1);
%     pY = f_pts_posed_c(i,2);
%     pZ = f_pts_posed_c(i,3);
%     
%     Sd(2*i-1:2*i) =  [pX/pZ pY/pZ]'; 
% end
% 
% [t,S] = ode45(@vs_dynamics,[0 2], S0);


%%



f_pt1_pose_w = f_pts_pose_w(1,:);

f_pt1_pose_c = [S(:,1).*S(:,2*num_points+1),...
                S(:,2).*S(:,2*num_points+1), ...
                S(:,2*num_points+1)];
            
cam_orientation = S(:,3*num_points+1:end);
cam_pose_w = compute_camera_pose(cam_orientation, f_pt1_pose_c, f_pt1_pose_w);




cam_x_t0 = cam_pose_w(1,1);
cam_y_t0 = cam_pose_w(1,2);
cam_z_t0 = cam_pose_w(1,3);

roll_t0 = cam_orientation(1,1);
pitch_t0 = cam_orientation(1,2);
yaw_t0 = cam_orientation(1,3);

cam_x_tf = cam_pose_w(end,1);
cam_y_tf = cam_pose_w(end,2);
cam_z_tf = cam_pose_w(end,3);

roll_tf = cam_orientation(end,1);
pitch_tf = cam_orientation(end,2);
yaw_tf = cam_orientation(end,3);

len = 5*ones(1,3);
center = [0 0 0];

figure(1);

for k = [1:1:size(cam_pose_w,1)-1,size(cam_pose_w,1)]
    
    plot_box(center, len);
    hold on;

    plot_camera([cam_pose_w(k,1) cam_pose_w(k,2) cam_pose_w(k,3)], ...
                [cam_orientation(k,1) cam_orientation(k,2) cam_orientation(k,3)], [2 2], 5);
        
    hold on;
    plot3(cam_pose_w(1:k,1), cam_pose_w(1:k,2), cam_pose_w(1:k,3), '--r');   
    
    

    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    grid on;
    
    xlim(gca,[-10 30])
    ylim(gca,[-30 10])
    zlim(gca, [-10 40])
    axis equal;
    view([60 26]);
    
    saveas(gcf, sprintf('picture%02d.png',k), 'png')
    clf;
    
end


% plot_camera([cam_x_t0 cam_y_t0 cam_z_t0], ...
%             [roll_t0 pitch_t0 yaw_t0], [2 2], 5);
% 
% plot_camera([cam_x_tf cam_y_tf cam_z_tf], ...
%             [roll_tf pitch_tf yaw_tf], [2 2], 5);  
  






f_pts_pose_w = ones(8,1)*[cx cy cz] + ([0 0 0;
                                 1 0 0; 
                                 1 1 0;
                                 0 1 0;
                                 0 1 1;
                                 1 1 1;
                                 1 0 1;
                                 0 0 1]-0.5*ones(8,3)).*(ones(8,1)*[lx ly lz]);
                             
f_pts_pose_t0_c = transform2camera_frame (cam_pose_w(1,:), cam_orientation(1,:), f_pts_pose_w);
f_pts_pose_t0_im = [f_pts_pose_t0_c(:,1)./f_pts_pose_t0_c(:,3) f_pts_pose_t0_c(:,2)./f_pts_pose_t0_c(:,3)];

f_pts_pose_tf_c = transform2camera_frame (cam_pose_w(end,:), cam_orientation(end,:), f_pts_pose_w);
f_pts_pose_tf_im = [f_pts_pose_tf_c(:,1)./f_pts_pose_tf_c(:,3) f_pts_pose_tf_c(:,2)./f_pts_pose_tf_c(:,3)];



figure
p = patch('Vertices',f_pts_pose_tf_im, ...
              'Faces',faces, ...
              'FaceColor', 'flat', ...
              'FaceVertexCData', cdata);
hold on;      
scatter(f_pts_pose_t0_im(:,1), f_pts_pose_t0_im(:,2),...
          'o', 'filled', ...
          'CData', fvcdata);
          grid on;
          axis equal;
          
hold on;      
scatter(f_pts_pose_tf_im(:,1), f_pts_pose_tf_im(:,2),...
          'o', 'filled', ...
          'CData', fvcdata);
          grid on;
          axis equal;     
          
hold on;
for i = 1:num_points
    plot(S(:,2*i-1),S(:,2*i),'--');
end