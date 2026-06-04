%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%  Minimax DDP Inverted Pendulum                  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%  Course: Advance Topics on Stochastic Optimal Control and Reinforcement Learning %%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%  AE8803 Spring 2014                             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%  Author: Wei Sun                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear all;
close all;


global m1;
global grav;
global l1;
global I1;
global b1;
global Q_f;
global Ru;
global Rv;
global kappa;

% Parameter
% masses in Kgr
 m1 = 1;

% damping coeff
 b1 = 0.1;

% length parameters in meters
 l1 = 0.5;
 
% Inertia in Kgr * m^2
 I1 = 0.25;

% Standard gravity
 grav = 9.81;

% Final Time
Tf = 1;
% Tf = 0.5;

% Number of steps for integral + 1
Horizon = 101; 

% Time step
 dt = Tf/(Horizon-1);

% Number of Iterations of Control Updates
num_iter = 20;
% num_iter = 10;

% Weight in Final State:
Q_f = zeros(2,2);
% Q_f(1,1) = 100;
% Q_f(2,2) = 10;
% Q_f(2,2) = 5;

% Weight in the Control u (Stablizer) :
Ru = 0.01;

% Weight in the Control v (Disstablizer) :
% Rv = 10;
Rv = 1;
% Rv = 0.2;
% Rv = 0.01;

% Initial Configuration:
xo = [pi 0];

% Target: 
p_target(1,1) = 0;
p_target(2,1) = 0;


% Learning Rate:
% gamma = 0.01;
% gamma = 0.4;
gamma = .8;
gamma = 1;

% kappa = 1 for 2nd order dynamics expansion, kappa = 0 for 1st order
% dynamics expansion.
% kappa = 1;
kappa = 0;

% Time for Iteration 
tt = linspace(0, Tf, Horizon);
tt_u = tt(1:(Horizon - 1));

% Initial Control:          
u = zeros(1,Horizon-1);
v = zeros(1,Horizon-1);

du = zeros(1,Horizon-1);
dv = zeros(1,Horizon-1);

u_cont = spline(tt_u,u);
v_cont = spline(tt_u,v);

% Initial trajectory:
options= odeset('OutputFcn','');
 [T, x_traj] = ode45('invertedpendulum_minimax', tt, xo, options, u_cont, v_cont);
 
 x_traj_cont(1) = spline(tt, x_traj(:, 1));
 x_traj_cont(2) = spline(tt, x_traj(:, 2));
 
 
%     figure()
%    subplot(2,2,1)
%    hold on
%    plot(tt,p_target(1,1)*ones(1,Horizon),'red','linewidth',4)
%    plot(tt,x_traj(:,1),'--b', 'linewidth',4);  
%    title('Theta','fontsize',20); 
%    xlabel('Time in sec','fontsize',20)
%    hold off;
%    grid;
%    
%    
%    subplot(2,2,2);hold on;
%    plot(tt,p_target(2,1)*ones(1,Horizon),'red','linewidth',4)
%    plot(tt,x_traj(:,2),'--b', 'linewidth',4); 
%    title('Theta dot','fontsize',20);
%    xlabel('Time in sec','fontsize',20)
%    hold off;
%    grid;
%    axis([0, 0.8, 0, 4])
%    
%    
%    subplot(2,2,3);hold on
%    plot(tt_u, u(1, :),'k', 'linewidth',2);
%    xlabel('Time in sec', 'fontsize',20)
%    title('Stabilizing Control', 'fontsize',20)
%    hold off;
%    grid;
%    
%    subplot(2,2,4);hold on
%    plot(tt_u, v(1, :),'k', 'linewidth',2);
%    xlabel('Time in sec', 'fontsize',20)
%    title('Destabilizing Control', 'fontsize',20)
%    hold off;
%    grid;
   

for k = 1:num_iter


%------------------------------------------------> Initial Condition of the
% Value function
Vxx_end= Q_f;
Vx_end = Q_f * (x_traj(end, :)' - p_target); 
V_end = 0.5 * (x_traj(end, :)' - p_target)' * Q_f * (x_traj(end, :)' - p_target); 

Vo = [V_end, Vx_end(:)', Vxx_end(:)'];

%------------------------------------------------> Integrate Backward the Value Function
tt_back = linspace(Tf, 0, Horizon);
[T, Vvalue] = ode45('ricattis_minimax', tt_back, Vo, options, u_cont, v_cont, x_traj_cont);
V = Vvalue(end:-1:1, 1);
Vx = Vvalue(end:-1:1, 2:3)';
for i = 1:Horizon
    Vxx(1:2,1,i) = Vvalue(end+1-i, 4:5);
    Vxx(1:2,2,i) = Vvalue(end+1-i, 6:7);
end

for i = 1:size(Vvalue, 2)
    V_cont(i) = spline(tt, Vvalue(end:-1:1, i));
end

%% 
%----------------------------------------------> Update the controls
Fu = [0; 1/I1];
Fv = [0; -1/I1];
dxo = [0; 0];
% du_cont = spline(tt_u,du);
[T, dx] = ode45('dxupdate_minimax', tt, dxo, options, u_cont, v_cont, x_traj_cont, V_cont);

for i = 1:(Horizon-1)
    
    dim_x = size(x_traj_cont, 2);
    dim_u = size(u_cont, 2);
    dim_v = size(v_cont, 2);
    
    x1_t = x_traj(i, 1);
    
    % pde of dynamics F
    Fx = [0, 1; m1 * grav * l1 * cos(x1_t)/I1,  - b1/I1];
    Fu = [0; 1/I1];
    Fv = [0; -1/I1];
    Fxx2 = [0, 0; -m1 * grav * l1 * sin(x1_t)/I1, 0];
    Fuu = zeros(dim_u);
    Fvv = zeros(dim_v);
    Fux = zeros(dim_u,dim_x);
    Fxu = zeros(dim_x,dim_u);
    Fvx = zeros(dim_v,dim_x);
    Fxv = zeros(dim_x,dim_v);
    Fuv = zeros(dim_u,dim_v);
    Fvu = zeros(dim_v,dim_u);
    
    % pde of running cost L
    L = (Ru*(u(i)^2) - Rv*(v(i)^2))/2;
    Lx = zeros(dim_x,1);
    Lu = Ru*u(i);
    Lv = -Rv*v(i);
    Lxx = zeros(dim_x);
    Luu = Ru;
    Lvv = -Rv;
    Lux = zeros(dim_u,dim_x);
    Lxu = zeros(dim_x,dim_u);
    Lvx = zeros(dim_v,dim_x);
    Lxv = zeros(dim_x,dim_v);
    Luv = zeros(dim_u,dim_v);
    Lvu = zeros(dim_v,dim_u);
    
    Qu = Fu'*Vx(:, i) + Lu;
    Qv = Fv'*Vx(:, i) + Lv;
    Quu = Luu + kappa*Fuu;
    Qvv = Lvv + kappa*Fvv;
    Qux = 1/2*(Lux + kappa*Fux) + 1/2*(Lxu + kappa*Fxu)' + Fu'*Vxx(:, :, i);
    Qvx = 1/2*(Lvx + kappa*Fvx) + 1/2*(Lxv + kappa*Fxv)' + Fv'*Vxx(:, :, i);
    Quv = 1/2*(Luv + kappa*Fuv) + 1/2*(Lvu + kappa*Fvu)';
    Qvu = Quv';
    
    lu(i) = -(Quu - Quv/Qvv*Qvu)\(Qu - Quv/Qvv*Qv);
    lv(i) = -(Qvv - Qvu/Quu*Quv)\(Qv - Qvu/Quu*Qu);
    Ku(:, i) = -(Quu - Quv/Qvv*Qvu)\(Qux - Quv/Qvv*Qvx);
    Kv(:, i) = -(Qvv - Qvu/Quu*Quv)\(Qvx - Qvu/Quu*Qux);
    
    du(i) = lu(i) + Ku(:, i)' * dx(i, :)';
    dv(i) = lv(i) + Kv(:, i)' * dx(i, :)';
    
    u_new(i) = u(i) + gamma*du(i);
    v_new(i) = v(i) + gamma*dv(i);
end

u = u_new;
v = v_new;

%%
%---------------------------------------------> Simulation of the Nonlinear System
% [x_traj] = fnsimulateDDP(xo,u_new,Horizon,dt,0);
% [Cost(:,k)] =  fnCostComputationDDP(x_traj,u_k,p_target,dt,Q_f,R);
% x1(k,:) = x_traj(1,:);

u_cont = spline(tt_u,u);
v_cont = spline(tt_u,v);
 [T, x_traj] = ode45('invertedpendulum_minimax', tt, xo, options, u_cont, v_cont);
 
 x_traj_cont(1) = spline(tt, x_traj(:, 1));
 x_traj_cont(2) = spline(tt, x_traj(:, 2));
 
[Cost(:,k)] =  fnCostComp_minimax(x_traj,u,v,p_target,dt);
 
fprintf('Iteration %d,  Current Cost = %e \n',k,Cost(1,k));
 


%    figure()
%    subplot(2,2,1)
%    hold on
%    plot(tt,p_target(1,1)*ones(1,Horizon),'red','linewidth',4)
%    plot(tt,x_traj(:,1),'--b', 'linewidth',4);  
%    title('Theta','fontsize',20); 
%    xlabel('Time in sec','fontsize',20)
%    hold off;
%    grid;
%    
%    
%    subplot(2,2,2);hold on;
%    plot(tt,p_target(2,1)*ones(1,Horizon),'red','linewidth',4)
%    plot(tt,x_traj(:,2),'--b', 'linewidth',4); 
%    title('Theta dot','fontsize',20);
%    xlabel('Time in sec','fontsize',20)
%    hold off;
%    grid;
%    axis([0, 0.8, -10, 0.01])
%    
%    subplot(2,2,3);hold on
%    plot(tt_u, u(1, :), 'k', 'linewidth',2);
%    xlabel('Time in sec', 'fontsize',20)
%    title('Stabilizing Control', 'fontsize',20)
%    hold off;
%    grid;
%    
%    subplot(2,2,4);hold on
%    plot(tt_u, v(1, :), 'k', 'linewidth',2);
%    xlabel('Time in sec', 'fontsize',20)
%    title('Destabilizing Control', 'fontsize',20)
%    hold off;
%    grid;
 
end

%% Plot section

   time(1)=0;
   for i= 2:Horizon
    time(i) =time(i-1) + dt;  
   end

      
%---------------------------------------------> Plot Section
   figure()
   subplot(2,2,1)
   hold on
   plot(time,x_traj(:,1), '--b',...
    'LineWidth',2);  
   plot(time,p_target(1,1)*ones(1,Horizon),'red','linewidth',4)
   title('Theta','fontsize',20); 
%    xlabel('Time in sec','fontsize',20)
   hold off;
   grid;
   
   
   subplot(2,2,2);hold on;
   plot(time,x_traj(:,2), '--b',...
    'LineWidth',2); 
   plot(time,p_target(2,1)*ones(1,Horizon),'red','linewidth',4)
   title('Theta dot','fontsize',20);
%    xlabel('Time in sec','fontsize',20)
   hold off;
   grid;
   
   
   subplot(2,2,3);hold on
   plot(time(1:end-1), u(1, :), 'linewidth',2);
%    xlabel('Time in sec', 'fontsize',20)
%    title(['Control u, Ru =', num2str(Ru)], 'fontsize',20)
   title('Stabilizing Control u', 'fontsize',20)
   
   subplot(2,2,4);hold on
   plot(tt_u, v(1, :), 'linewidth',2);
%    xlabel('Time in sec', 'fontsize',20)
   title('Destabilizing Control v', 'fontsize',20)
   hold off;
   
   
   figure()
   hold on
   set(gca, 'fontsize',15);
   aa = plot(time(1:end-1), u(1, :),'b','linewidth',2);
   ab = plot(tt_u, v(1, :), 'k','linewidth',2);
   legend([aa,ab],{'$\theta$','$\dot{\theta}$'},'Interpreter','latex')
   
   figure()
   hold on
   plot(Cost, '--rs',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor', 'y'); 
   xlabel('Iterations','fontsize',20)
   title('Cost','fontsize',20);
   
%    figure(21)
%    subplot(2,2,3);hold on
%    plot(time(1:end-1), u(1, :)-v(1, :), '--y', 'linewidth',2);
%    xlabel('Time in sec', 'fontsize',20)
%    title(['Control u, Ru =', num2str(Ru)], 'fontsize',20)
%    title('Stabilizing Control u', 'fontsize',20)
   
   
%   Case: kappa = 1
if kappa == 1
   figure(50)
   hold on
   plot(Cost, 'g', 'linewidth',2); 
   xlabel('Iterations','fontsize',20)
   title('Cost','fontsize',20);
   
   figure(51)
   hold on
   plot(time(1:end-1), 0*time(1:end-1), 'b', 'linewidth', 2)
   plot(time(1:end-1), u(1, :), 'g', 'linewidth',2);
   xlabel('Time in sec', 'fontsize',20)
   title('Control1', 'fontsize',20)
end
   
% Case: kappa = 0
if kappa == 0
   figure(50)
   hold on
   plot(Cost, '-.r', 'linewidth',2); 
   xlabel('Iterations','fontsize',20)
   title('Cost','fontsize',20);
   
   figure(51)
   hold on
   plot(time(1:end-1), u(1, :), '-.r', 'linewidth',2);
   xlabel('Time in sec', 'fontsize',20)
   title('Control1', 'fontsize',20)
end
   
    %% Apply control on stochastic dynamics


%     %% Mean and  Variation on Stochastic System
% clear x_star_mean x_star_std x_star u_new
% % sigma = 0.5;
% % sigma = 1;
% % sigma = 2;
% sigma = 4;
% % sigma = 10;
% iterations = 1e3;
% % iterations = 1e4;
% % iterations = 1e5;
% 
% for j = 1:iterations
%     x_star(:, 1, j)= zeros(2,1);
%     x_star(1, 1, j)= pi;
%     for k=1:(Horizon-1)
%         F_x(1,1) = x_star(2,k,j);
%         F_x(2,1) = m1 * grav * l1 * sin(x_star(1,k,j))/I1 - b1*x_star(2,k,j)/I1;
%         
%         G_x(1,1) = 0;
%         G_x(2,1) = 1/I1;
%         % u_new(:,k) = u(:,k) + lu(k) + Ku(:,k)'*(x_star(:,k) - x_traj(k,:)');
%         u_new(:,k) = u(:,k) + Ku(:,k)'*(x_star(:,k,j) - x_traj(k,:)');
%         
%         x_star(:,k+1,j) = x_star(:,k,j) + F_x * dt + G_x * u_new(:,k) * dt  + sigma * [0; x_star(1,k,j)] * randn(1)* sqrt(dt) ;
%     end
% end
% 
% % l = 1;
% % t_iter(1) = 0;
% % M = 10;
% % for k = 1:Horizon
% %     if mod(k,M)==1
% %         x_star_mean(1,l) = mean(x_star(1,k,:));
% %         x_star_mean(2,l) = mean(x_star(2,k,:));
% %         x_star_std(1,l) = std(x_star(1,k,:));
% %         x_star_std(2,l) = std(x_star(2,k,:));
% %         l = l+1;
% %         t_iter(l) = t_iter(l-1) + M * dt;
% %     end
% % end
% % 
% % t_iter(end) = [];
% % 
% %    figure()
% %    figs = [];
% %    subplot(2,2,1)
% %    hold on
% %    errorbar(t_iter,x_star_mean(1,:),x_star_std(1,:));  
% %    plot(time,p_target(1,1)*ones(1,Horizon),'red','linewidth',2)
% %    title('Theta','fontsize',20); 
% %    xlabel('Time in sec','fontsize',20)
% %    hold off;
% %    grid;
% %    figs = [figs, subplot(2,2,1)];
% %    
% %    
% %    subplot(2,2,2);hold on;
% %    errorbar(t_iter,x_star_mean(2,:),x_star_std(2,:)); 
% %    plot(time,p_target(2,1)*ones(1,Horizon),'red','linewidth',2)
% %    title('Theta dot','fontsize',20);
% %    xlabel('Time in sec','fontsize',20)
% %    hold off;
% %    grid;
% %    figs = [figs, subplot(2,2,2)];
% %    
% % %    set(figs, 'fontsize', 15)
% %    suptitle(['Inverted Pendulum Minimax DDP, sigma = ', num2str(sigma)])
%    
%    for k = 1:Horizon
%         x_star_mean(1,k) = mean(x_star(1,k,:));
%         x_star_mean(2,k) = mean(x_star(2,k,:));
%         x_star_std(1,k) = std(x_star(1,k,:));
%         x_star_std(2,k) = std(x_star(2,k,:));
%    end
% 
%    lw = 2.5;
% %    figure()
% %    figs = [];
% %    subplot(2,2,1)
%    figure(1)
%    hold on
%    h = errorbar(time,x_star_mean(1,:),x_star_std(1,:));  
% %    hc = get(h, 'Children'); set(hc(1),'Linewidth',lw)
%    plot(time,p_target(1,1)*ones(1,Horizon),'red','linewidth',2)
%    title('Theta','fontsize',20); 
%    xlabel('Time in sec','fontsize',20)
%    hold off;
%    grid;
% %    figs = [figs, subplot(2,2,1)];
%    
% %    subplot(2,2,2);
%    figure(2)
%    hold on;
%    h = errorbar(time,x_star_mean(2,:),x_star_std(2,:)); 
% %    hc = get(h, 'Children'); set(hc(1),'Linewidth',lw)
%    plot(time,p_target(2,1)*ones(1,Horizon),'red','linewidth',2)
%    title('Theta dot','fontsize',20);
%    xlabel('Time in sec','fontsize',20)
%    hold off;
%    grid;
% %    figs = [figs, subplot(2,2,2)];
% %    
% % %    set(figs, 'fontsize', 15)
% %    suptitle(['Inverted Pendulum Minimax DDP, \sigma = ', num2str(sigma), ', Q_f = diag', '\{',  num2str(Q_f(1,1)), ',', num2str(Q_f(2,2)), ...
% %        '\}, Ru = ', num2str(Ru), ', Rv = ', num2str(Rv)])
% 
% %% 
% % plot eigenvalue of Vxx
% for i = 1:size(Vxx,3)
%     eig_value(:,i) = eig(Vxx(:,:,i));
% end
% 
% figure(3)
% subplot(2,1,1)
% hold on
% plot(eig_value(1,:), 'b')
% subplot(2,1,2)
% hold on
% plot(eig_value(2,:), 'b')
% 
% % plot feeback gain per time step
% figure(4)
% subplot(2,1,1)
% hold on
% plot(Ku(1,:), 'b')
% subplot(2,1,2)
% hold on
% plot(Ku(2,:), 'b')
% 
% % plot control u
% figure(5)
% hold on
% plot(u(1,:), 'b')
   