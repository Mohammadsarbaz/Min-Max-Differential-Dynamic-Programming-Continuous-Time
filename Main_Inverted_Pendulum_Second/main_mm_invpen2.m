clc;
clear;
close all;

% Select case_num for different value of Rv, case 1. Rv = 0.13, case 2. Rv = 0.2, case 3. Rv = 10, case 4. Rv = 1.
% case_num = 1;
% case_num = 2;
% case_num = 3;
case_num = 4;

global m1;
global grav;
global l1;
global I1;
global b1;
global Q_f Q p_target;
global Ru;
global Rv;
global kappa;
global a;

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
 
% scale parameter % G = [0; a*x(1)];
 a = 1; 
%  a = 6;  

% Final Time
Tf = 1;
% Tf = 0.8;
% Tf = 0.6;

% Number of steps for integral + 1
Horizon = 101; 

% Time step
 dt = Tf/(Horizon-1);

% Number of Iterations of Control Updates
num_iter = 20;

% Weight in Final State:
Q_f = zeros(2,2);
% Q_f(1,1) = 100;
% Q_f(2,2) = 10;
% Q_f(2,2) = 5;

Q = zeros(2);
% Q = diag([10,1]);
Q = diag([1,.1]);

% Q_f = 10*Q;

% Weight in the Control u (Stablizer) :
% Ru = 0.1;
Ru = 0.01;

% Weight in the Control v (Disstablizer) :
switch case_num
    case 1
        Rv = 0.13;
    case 2
        Rv = 0.2;
    case 3
        Rv = 10;
    case 4
        Rv = 1;
end
% Rv = 0.2;
% Rv = 1;
% Rv = 0.01;

% Initial Configuration:
xo = [pi 0];

% Target: 
p_target=zeros(2,1);
p_target(1,1) = 0;
p_target(2,1) = 0;


% Learning Rate:
% gamma = 0.01;
% gamma = 0.5;
gamma = .8;

% kappa = 1 for 2nd order dynamics expansion, kappa = 0 for 1st order
% dynamics expansion.
% kappa = 1;
kappa = 0;

% Time for Iteration 
tt = linspace(0, Tf, Horizon);
tt_u = tt(1:(Horizon - 1));

% Initial Control:          
u = zeros(1,Horizon-1);
du = zeros(1,Horizon-1);

v = zeros(1,Horizon-1);
dv = zeros(1,Horizon-1);

u_cont = spline(tt_u,u);
v_cont = spline(tt_u,v);

% Initial trajectory:
options= odeset('OutputFcn','');
 [T, x_traj] = ode45('invertedpendulum_minimax', tt, xo, options, u_cont, v_cont);
 
 x_traj_cont(1) = spline(tt, x_traj(:, 1));
 x_traj_cont(2) = spline(tt, x_traj(:, 2));

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
% V = Vvalue(end:-1:1, 1);
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
%     L = (Ru*(u(i)^2) - Rv*(v(i)^2))/2;
%     Lx = zeros(dim_x,1);
    Lu = Ru*u(i);
    Lv = -Rv*v(i);
%     Lxx = zeros(dim_x);
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
%    plot(tt,x_traj(:,1),'linewidth',4);  
%    plot(tt,p_target(1,1)*ones(1,Horizon),'red','linewidth',4)
%    title('Theta','fontsize',20); 
%    xlabel('Time in sec','fontsize',20)
%    hold off;
%    grid;
%    
%    
%    subplot(2,2,2);hold on;
%    plot(tt,x_traj(:,2),'linewidth',4); 
%    plot(tt,p_target(2,1)*ones(1,Horizon),'red','linewidth',4)
%    title('Theta dot','fontsize',20);
%    xlabel('Time in sec','fontsize',20)
%    hold off;
%    grid;
%    
%    
%    subplot(2,2,3);hold on
%    plot(tt_u, u(1, :), 'linewidth',2);
%    xlabel('Time in sec', 'fontsize',20)
%    title('Stabilizing Control', 'fontsize',20)
%    hold off;
%    
%    subplot(2,2,4);hold on
%    plot(tt_u, v(1, :), 'linewidth',2);
%    xlabel('Time in sec', 'fontsize',20)
%    title('Destabilizing Control', 'fontsize',20)
%    hold off;
 
end

   time(1)=0;
   for i= 2:Horizon
    time(i) =time(i-1) + dt;  
   end

 %%     
%---------------------------------------------> Plot Section

   time(1)=0;
   for i= 2:Horizon
    time(i) =time(i-1) + dt;  
   end

 figure()
   subplot(2,2,[1 2])
   hold on
   set(gca,'fontsize',15)
   aa = plot(time,x_traj(:,1)','b','linewidth',2);  
   ab = plot(time,x_traj(:,2)','--k','linewidth',2); 
   ac = plot(time,p_target(1,1)*ones(1,Horizon),'red','linewidth',3);
%    str=sprintf('Trajectories, dt = %.3f, Rv = %.3f', E.dt, E.cv);
%    title(str, 'fontsize',20); 
   axis([0 1 -10 5])
   xlabel('Time [sec]')
   ylabel('x')
   legend([aa,ab,ac],{'$\theta$','$\dot{\theta}$','goal states'},'Interpreter','latex')
   hold off;
   grid;
   
   
% %    subplot(2,2,2);hold on;
% %    plot(time,p_target(2,1)*ones(1,Horizon),'red','linewidth',4)
% %    title('Theta dot','fontsize',20);
% % %    xlabel('Time in sec','fontsize',20)
% %    hold off;
% %    grid;
%    
   
   subplot(2,2,3);hold on
   set(gca,'fontsize',15)
   plot(time(1:end-1), u(1, :), 'linewidth',2);
   xlabel('Time [sec]')
   ylabel('u [N]')
%    title('Control u', 'fontsize',20)
   axis([0, 1, -30, 10])
   
   subplot(2,2,4);hold on
   set(gca,'fontsize',15)
   plot(time(1:end-1), -v(1, :), 'linewidth',2);
   xlabel('Time [sec]')
   ylabel('v [N]')
%    title('Control v', 'fontsize',20)
   axis([0, 1, -0.1, 0.3])
   hold off;


%% 
% % plot eigenvalue of Vxx
% for i = 1:size(Vxx,3)
%     eig_value(:,i) = eig(Vxx(:,:,i));
% end
% 
% figure(3)
% subplot(2,1,1)
% hold on
% plot(eig_value(1,:), 'r--')
% subplot(2,1,2)
% hold on
% plot(eig_value(2,:), 'r--')
% 
% % plot feeback gain per time step
% figure(4)
% subplot(2,1,1)
% hold on
% plot(Ku(1,:), 'r--')
% subplot(2,1,2)
% hold on
% plot(Ku(2,:), 'r--')
% 
% % plot control u
% figure(5)
% hold on
% plot(u(1,:), 'r--')
   