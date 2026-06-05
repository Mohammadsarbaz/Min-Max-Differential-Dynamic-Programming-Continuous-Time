function MovieFile = PlotResults(sol,ianim,imov)

if ianim == 0 || isempty(ianim)
    imov = 0;
end


x_traj = sol.state;
time = sol.time;
u = sol.control_u;
v = sol.control_v;
Cost = sol.cost;
p_target = sol.auxdata.p_target;
Ku = sol.Ku;
Kv = sol.Kv;
lu = sol.lu;
lv = sol.lv;

figure('Position',[100, 100, 1280, 720]);
Horizon = 501;
for i = 1:16
    subplot(4,4,i)
    hold on
    plot(time,x_traj(i,:),'linewidth',2);
    plot(time,p_target(i,1)*ones(1,Horizon),'--r','linewidth',2)
    set(gca,'FontSize',15)
    title(['x_{', num2str(i), '}'],'fontsize',15);
    xlabel('Time [sec]','fontsize',15)
    hold off;
    grid;
end

figure()
for i = 1:4
    subplot(2,2,i);hold on
    plot(time(1:end), u(i, :), 'linewidth',2);
    set(gca,'FontSize',15)
    xlabel('Time [sec]', 'fontsize',15)
    title(['u(', num2str(i), ') [rpm]'], 'fontsize',15)
    xlim([0, 2])
end

figure()
for i = 1:4
    subplot(2,2,i);hold on
    plot(time(1:end), v(i, :), 'linewidth',2);
    set(gca,'FontSize',15)
    xlabel('Time in sec', 'fontsize',15)
    title(['v(', num2str(i), ') [rpm]'], 'fontsize',15)
end
   
figure()
set(gca,'fontsize',15)
hold on
plot(Cost,'linewidth',2); 
xlabel('Iterations')
title('Cost');
grid;
   
%print(gcf,'DDP_figure.png','-dpng')

% Plot controller gains
figure('Position',[600 100 524 564]);
[n1,n2,n3] = size(Ku);
for i = 1:n1
     for j=1:n2
        subplot(n2,n1,i+(j-1)*n1);
        plot(time,squeeze(Ku(i,j,:)),'linewidth',2);
        xlabel('Time [sec]', 'fontsize',15)
        ylabel(['K(' num2str(i) ',' num2str(j) ')'], 'fontsize',15)
        grid;
     end
end
title('Controller u Gains')
  
% if ianim == 0 || isempty(ianim)
%     MovieFile = [];
%     return 
% end 

MovieFile = [];
   
  %  animation
  
%   Res.x = x_traj(1,:);
%   Res.theta = x_traj(3,:);
%   Res.t = time;
%   [MovieFile,tanim,x_i,theta_i] = replay_anim(Res,0.02,1,1,imov);

x_traj = x_traj';

%%
quadrotor_trajectory_plotting;
% keyboard()
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% function [Movie,tanim,x_i,theta_i] = replay_anim(Res,DispDelay,scale,refine,imov)
% 
% global CartPole
% 
% parameters;
% 
% if isempty(DispDelay), DispDelay = 0.05; end
% if isempty(scale), scale = 1; end
% if isempty(imov), imov = 0; end
% 
% animfig = figure;
% set(animfig,'DoubleBuffer','on','position',[100 200 624 464]);
% %set(animfig,'DoubleBuffer','on','position',[1 31 1024 664],'Renderer','zbuffer');
% 
% % Initialization
% 
% %==========================================================================
% XYaxes = axes;
% set(XYaxes,'color',[0.8 0.8 0.8 ],'DataAspectRatio',[1 1 1]);
% axis equal
% %==========================================================================
% 
% if isempty(Res)
%     return
% end
% 
% % Horizon
% line('XData',[-2 2],'YData',[0 0],'color','k','LineStyle','-','LineWidth',2);
% 
% % Initial Positions of Cart and Pole
% Cart = rectangle('Position',[Res.x(1)-0.15, -0.1, 0.3, 0.2],'FaceColor',[0, 0.5, 0.5], 'EdgeColor', 'r',...
%              'LineStyle','-','LineWidth',3);
% Pole = line('Xdata',[Res.x(1) Res.x(1)+CartPole.l1*sin(Res.theta(1))],...
%              'Ydata',[0 CartPole.l1*cos(Res.theta(1))],...
%              'color','b','LineStyle','-','LineWidth',7);
%          
% % Running Time
% Time = text(-0.5,0.05,['Time = ',num2str(0,'%-5.2f'),' (sec)'],'FontSize',14); 
% 
% 
% disp('Hit any key to continue...')
% pause;
% 
% 
% tfinal = Res.t(length(Res.t));
% 
% if tfinal>0
%   tanim = linspace(0,tfinal,100*refine);
% else
%   tanim = linspace(tfinal,0,100*refine);
% end
% 
% 
% x_i = interp1(Res.t,Res.x,tanim);
% theta_i = interp1(Res.t,Res.theta,tanim);
% 
% if imov == 1
%    Movie(1:length(tanim)) = struct('cdata',[],'colormap',[]);
% else
%    Movie = []; 
% end
% 
% 
% for j = 1:length(tanim) 
%     
%    set(Cart,'Position',[x_i(j)-0.15, -0.1, 0.3, 0.2]);
%    set(Pole,'Xdata',[x_i(j) x_i(j)+CartPole.l1*sin(theta_i(j))],...
%              'Ydata',[0 CartPole.l1*cos(theta_i(j))]);
%    set(Time,'string',['Time = ',num2str(tanim(j),'%-5.2f'),' (sec)']);
%          
%     drawnow
%     pause(DispDelay)
%     
%     
% if imov == 1
%    Movie(j) = getframe(gcf);
% end
% 
% 
% end
% 
% 
% end




  