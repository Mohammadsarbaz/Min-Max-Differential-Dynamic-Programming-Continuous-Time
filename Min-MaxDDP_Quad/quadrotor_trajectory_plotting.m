%
%   Oktay Arslan, 2015
%


%%% run this code after executing "main_cont_Quadrotor_state_cost.m"


d0 = pwd;
addpath([d0,'/plotting']);

%%
%clear all;
%imu=importdata('./data/sfly/imu_1loopDown.txt',' ',3);
%vicon = importdata('./data/sfly/vicon_1loopDown.txt',' ',3);

%imu_acc=struct('x',imu.data(:,2),'y',imu.data(:,3),'z',imu.data(:,4));

%vicon_pose =struct('x',vicon.data(:,2),'y',vicon.data(:,3),'z',vicon.data(:,4));
vicon_pose =struct('x',x_traj(:,1),'y',x_traj(:,2),'z',x_traj(:,3));

vicon_angle = struct('roll',x_traj(:,4),'pitch',x_traj(:,5),'yaw',x_traj(:,6));
%time=struct('imu',imu.data(:,1),'vicon',vicon.data(:,1));

%time.vicon=time.vicon-time.vicon(1);
%time.imu = time.imu-time.imu(1);


%%%%%%

sf = 0.25; % scaling factor
cone.base_position = [0 0 0];
cone.base_radius = sf * 0.05;
cone.top_radius = 0;

cylinder.base_position = [0 0 0];
cylinder.base_radius = sf * 0.025;
cylinder.top_radius = sf * 0.025;

offset = [0 0 0];
length = sf * [0.5 0.5 0.5];
body_ratio = sf * [0.8 0.8 0.8];

in_args.cone = cone;
in_args.cylinder = cylinder;
in_args.Npoints = 10;
in_args.offset = offset;
in_args.length =length;
in_args.body_ratio = body_ratio;



%%%%%%%

length = 0.1;

%aviobj = avifile('test.avi');
scrsz = get(0,'ScreenSize');
%ScreenSize is a four-element vector: [left, bottom, width, height]:

fig=figure('Position',[1 scrsz(4)/2 scrsz(3)/2 scrsz(4)/2]);
title_handle = title('Quadrotor 6DOF coordinates plot');


itv=5;
% rotation_spd=0.4;
rotation_spd=0;
delay=0.01;

az=15;
el=64;
% az=55;
% el=15;
view(az,el);
grid on;
xlabel('x', 'fontsize',16);
ylabel('y', 'fontsize',16);
zlabel('z', 'fontsize',16);
h_legend=legend('X','Y','Z');

%% prepare for movie
% count=1;
% for i = 1:itv:numel(vicon_angle.roll)
%     
%     figure(7);
%     clf;
% 
% 
%     el=20;
% 
%     position = [vicon_pose.x(i),vicon_pose.y(i),vicon_pose.z(i)];
%     orientation = [vicon_angle.roll(i),vicon_angle.pitch(i),vicon_angle.yaw(i)];
% 
%     %%% plot inertial frame
%     plot_frame ([0 0 0], [0 0 0],in_args);
%     
%     color_vec = [1 0.6 0];
%     plot_quadrotor_new(p_target(1:3)', p_target(4:6)', color_vec);
% 
%     plot_quadrotor_new(position, orientation);
% 
%     hold on;
%     plot3(vicon_pose.x(1:i)',vicon_pose.y(1:i)',vicon_pose.z(1:i)', '-r');
% 
% % % % % %     % From peter's toolbox
% % % % % %     R = rpy2r(vicon_angle.roll(i),vicon_angle.pitch(i),vicon_angle.yaw(i));
% % % % % % 
% % % % % %     % generate axis vectors
% % % % % %     tx = [length,0.0,0.0];
% % % % % %     ty = [0.0,length,0.0];
% % % % % %     tz = [0.0,0.0,length];
% % % % % %     % Rotate it by R
% % % % % %     t_x_new = R*tx';
% % % % % %     t_y_new = R*ty';
% % % % % %     t_z_new = R*tz';
%     
%     
%     
%     % translate vectors to camera position. Make the vectors for plotting
% % % % % %     origin = [vicon_pose.x(i),vicon_pose.y(i),vicon_pose.z(i)];
%     
% % % % % %     tx_vec(1,1:3) = origin;
% % % % % %     tx_vec(2,:) = t_x_new + origin';
% % % % % % 
% % % % % %     ty_vec(1,1:3) = origin;
% % % % % %     ty_vec(2,:) = t_y_new + origin';
% % % % % %     
% % % % % %     tz_vec(1,1:3) = origin;
% % % % % %     tz_vec(2,:) = t_z_new + origin';
%     
%     
%     hold on;
%     
%     % Plot the direction vectors at the point
% % % % % %     p1=plot3(tx_vec(:,1), tx_vec(:,2), tx_vec(:,3));
% % % % % %     set(p1,'Color','Green','LineWidth',1);
% % % % % %     p1=plot3(ty_vec(:,1), ty_vec(:,2), ty_vec(:,3));
% % % % % %     set(p1,'Color','Blue','LineWidth',1);
% % % % % %     p1=plot3(tz_vec(:,1), tz_vec(:,2), tz_vec(:,3));
% % % % % %     set(p1,'Color','Red','LineWidth',1);
%     
%     perc = count*itv/numel(vicon_angle.roll)*100;
%     %%%%fprintf('Process = %f\n',perc);
%     %%%%%text(1,-3,0,['Process = ',num2str(perc),'%']);
%     set(title_handle,'String',['Process = ',num2str(perc),'%'],'fontsize',16);
%        
%     
%     
%     %plot3(p_target(1,1),p_target(2,1),p_target(3,1),'Marker', 'o', 'MarkerFaceColor', 'red');
% 
%     az=az+rotation_spd;
%     view(az,el);
%     %drawnow;
% 
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
% 
%     axis equal;
%     grid on;
%     box on;
% 
%     xlim([-2.5 2.5]);
%     ylim([-2.5 2.5]);
%     zlim([-2 2]);
%     
%     xlim([-2 6]);
%     ylim([-2 6]);
%     zlim([0 2]);
% 
%     pause(delay);  % in second
%     
%     filename = sprintf('./figures/fig_quadrotor_%04d.png',count);
%     print('-dpng',filename);
%     
%     
% %     % Store the frame
%     Mov(count)=getframe(gcf); % leaving gcf out crops the frame in the movie.
% 	
%     count=count+1;
%     
%     %f = getframe(fig);
%     %aviobj=addframe(aviobj,f);
%     
% end;
% keyboard()

%% generate snapshots
count=1;
for i = 1:numel(vicon_angle.roll)
    
%     if i == 1 || i == 250 || i == 350 || i == 500 % travel
    if i == 150 || i == 250 || i == 500 % pitch 2pi
    figure(17);
%     clf;


    el=20;

    position = [vicon_pose.x(i),vicon_pose.y(i),vicon_pose.z(i)];
    orientation = [vicon_angle.roll(i),vicon_angle.pitch(i),vicon_angle.yaw(i)];

    %%% plot inertial frame
%     plot_frame ([0 0 0], [0 0 0],in_args);
    color_vec = [1 0.6 0];
%     plot_quadrotor_new(p_target(1:3)', p_target(4:6)', color_vec, sf);

    plot_quadrotor_new(position, orientation, color_vec, sf);

    hold on;
    plot3(vicon_pose.x(1:i)',vicon_pose.y(1:i)',vicon_pose.z(1:i)', '-r');

% % % % %     % From peter's toolbox
% % % % %     R = rpy2r(vicon_angle.roll(i),vicon_angle.pitch(i),vicon_angle.yaw(i));
% % % % % 
% % % % %     % generate axis vectors
% % % % %     tx = [length,0.0,0.0];
% % % % %     ty = [0.0,length,0.0];
% % % % %     tz = [0.0,0.0,length];
% % % % %     % Rotate it by R
% % % % %     t_x_new = R*tx';
% % % % %     t_y_new = R*ty';
% % % % %     t_z_new = R*tz';
    
    
    
    % translate vectors to camera position. Make the vectors for plotting
% % % % %     origin = [vicon_pose.x(i),vicon_pose.y(i),vicon_pose.z(i)];
    
% % % % %     tx_vec(1,1:3) = origin;
% % % % %     tx_vec(2,:) = t_x_new + origin';
% % % % % 
% % % % %     ty_vec(1,1:3) = origin;
% % % % %     ty_vec(2,:) = t_y_new + origin';
% % % % %     
% % % % %     tz_vec(1,1:3) = origin;
% % % % %     tz_vec(2,:) = t_z_new + origin';
    
    
    hold on;
    
    % Plot the direction vectors at the point
% % % % %     p1=plot3(tx_vec(:,1), tx_vec(:,2), tx_vec(:,3));
% % % % %     set(p1,'Color','Green','LineWidth',1);
% % % % %     p1=plot3(ty_vec(:,1), ty_vec(:,2), ty_vec(:,3));
% % % % %     set(p1,'Color','Blue','LineWidth',1);
% % % % %     p1=plot3(tz_vec(:,1), tz_vec(:,2), tz_vec(:,3));
% % % % %     set(p1,'Color','Red','LineWidth',1);
    
    perc = count*itv/numel(vicon_angle.roll)*100;
    %%%%fprintf('Process = %f\n',perc);
    %%%%%text(1,-3,0,['Process = ',num2str(perc),'%']);
    set(title_handle,'String',['Process = ',num2str(perc),'%'],'fontsize',16);
    
%     set(gcf, 'position', [100, 100, 1280, 720])
       
    
    
    %plot3(p_target(1,1),p_target(2,1),p_target(3,1),'Marker', 'o', 'MarkerFaceColor', 'red');

    az=az+rotation_spd;
    view(az,el);
    %drawnow;

    xlabel('x');
    ylabel('y');
    zlabel('z');

    axis equal;
    grid on;
    box off;

    xlim([-1 1]);
    ylim([-1 1]);
    zlim([-0.5 1]);
    
%     xlim([-2 6]);
%     ylim([-2 6]);
%     zlim([0 2]);

%     xlim([-2 5]);
%     ylim([-2 5]);
%     zlim([-2 2]);

    pause(delay);  % in second
    
%     filename = sprintf('./figures/fig_quadrotor_%04d.png',count);
%     print('-dpng',filename);
    
    
%     % Store the frame
%     Mov(count)=getframe(gcf); % leaving gcf out crops the frame in the movie.
	
    count=count+1;
    
    %f = getframe(fig);
    %aviobj=addframe(aviobj,f);
%     keyboard()
    
    end 
end

%aviobj=close(aviobj);

fprintf('Done\n');

%%
% writerObj = VideoWriter('animation_qr1.avi');
% % writerObj = VideoWriter('PE_gcircle2.avi');
% writerObj.FrameRate=2;
% open(writerObj);
% for i=1:(count-1)
%   filename = sprintf('./figures/fig_quadrotor_%04d.png',i);
%   thisimage = imread(filename);
%   writeVideo(writerObj, thisimage);
% end
% close(writerObj);




