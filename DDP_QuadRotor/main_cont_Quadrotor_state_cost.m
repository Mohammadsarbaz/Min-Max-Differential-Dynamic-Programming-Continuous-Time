clc;
clear all;
close all;

%%%%% Make global so these can be used elsewhere
global m grav k_F k_M k_m wh L I invI wmax wmin G numStates numControls Q_f R kappa deltax Q p_target

%%%%% Set up physical properties of the system and adjust initial state
numStates = 16;     %%%%%  SET UP NUMBER OF STATES  %%%%%
numControls = 4;    %%%%% SET UP NUMBER OF CONTROLS %%%%%

%%%%% Quadrotor Physical Properties
m = 0.5;       % mass in kg
grav = 9.81;    % gravitational acceleration in m/s^2
k_F = 6.11E-8;   % force motor constant in N/(r/min^2)
k_M = 1.5E-9;    % moment motor constant in Nm/(r/min^2)
k_m = 1/20;
wh = ((m*grav/4)/k_F)^0.5;   % rotor speed to hold hover in r/min
L = 0.38;       % length of moment arm to rotors in m
Ixx = 3.9E-3;
Iyy = 4.4E-3;
Izz = 4.9E-3;
I = [Ixx 0 0;0 Iyy 0; 0 0 Izz]; % mass moment of inertia matrix
invI = inv(I);  % inverse of mass moment of inertia matrix
wmax = 7800;    % r/min
wmin = 1200;    % r/min
G = zeros(16,4);
G(13:16, :) = k_m.*[1 0 -1 1; ...
    1 1 0 -1; ...
    1 0 1 1; ...
    1 -1 0 -1]; % Fu, partial(F)/partial(u)

% Final Time
% Tf = 5;
Tf = 2;
% Tf = 0.5;

dt0 = 0.001;

% Number of steps for integral + 1
% Horizon = 151; 
Horizon = floor(Tf/dt0) + 1;

% Time step
 dt = Tf/(Horizon-1);


%% Final State Weights
Q_f = zeros(numStates);

%%%%% Manually choose these values

%% Terminal Cost Q_f choice 1

% position states
Q_f(1,1) = 10000000;
Q_f(2,2) = 10000000;
Q_f(3,3) = 10000000;
% angular states
Q_f(4,4) = 100000;
Q_f(5,5) = 100000;
Q_f(6,6) = 100000;
% velocity states
% Q_f(7,7) = 1000;
% Q_f(8,8) = 1000;
% Q_f(9,9) = 1000;
Q_f(7,7) = 1e6;
Q_f(8,8) = 1e6;
Q_f(9,9) = 1e6;
% angular velocity states
Q_f(10,10) = 1000;
Q_f(11,11) = 1000;
Q_f(12,12) = 1000;
% motor states
Q_f(13,13) = 0;
Q_f(14,14) = 0;
Q_f(15,15) = 0;
Q_f(16,16) = 0;

% %% Terminal Cost Q_f choice 2
% 
% % % Q_f in continuous DDP
% for i = 1:12
% Q_f(i, i) = 1e8; % p_target(3,1) = 1;
% end
% 
%% Terminal Cost Q_f choice 3

% for i = 1:12
% Q_f(i, i) = 1e6; 
% end

%% Terminal Cost Q_f choice 4, for take-off

% for i = 1:12
% Q_f(i, i) = 1e6; 
% end
% Q_f(3, 3) = 1e8;

%% Terminal Cost Q_f choice 5, for pitch change of 2pi

% for i = 1:3
% Q_f(i, i) = 1e7; 
% end
% for i = 4:6
% Q_f(i, i) = 1e5; 
% end
% for i = 7:9
% Q_f(i, i) = 1e5; 
% end
% for i = 10:12
% Q_f(i, i) = 1e5; 
% end
% Q_f(5, 5) = 1e7;

%% Terminal Cost Q_f choice 6, for pitch change of 2pi, works with gamma 0.5, num_iter 50

% for i = 1:3
% Q_f(i, i) = 1e7; 
% end
% for i = 4:6
% Q_f(i, i) = 1e6; 
% end
% for i = 7:9
% Q_f(i, i) = 1e6; 
% end
% for i = 10:12
% Q_f(i, i) = 1e5; 
% end
% Q_f(5, 5) = 1e7;

%% State Weights
Q = 0.00001*Q_f;
% Q = 1e-3*Q_f;

%% Control Weights
R = 0.0001*eye(numControls);

%% Target States

%%%%% Manually choose these values
% position target
p_target(1,1) = 4;
p_target(2,1) = 0;
p_target(3,1) = 1;
% p_target(3,1) = 1;

% angular target
% p_target(4,1) = pi/15;  % roll
% p_target(5,1) = pi/12;  % pitch
p_target(6,1) = pi;  % yaw
p_target(4,1) = 0;  % roll
p_target(5,1) = 0;  % pitch
% p_target(6,1) = 0;  % yaw

% velocity target
p_target(7,1) = 0;
p_target(8,1) = 0;
p_target(9,1) = 0;

% angular velocity target
p_target(10,1) = 0;
p_target(11,1) = 0;
p_target(12,1) = 0;

% motor target
p_target(13,1) = 0;
p_target(14,1) = 0;
p_target(15,1) = 0;
p_target(16,1) = 0;

%%%%% Initial State
xo = zeros(16,1);
% xo(7,1) = 5;
% xo(9,1) = 5;
% xo(12,1) = 0.5;
xo(13,1) = wh;
xo(14,1) = wh;
xo(15,1) = wh;
xo(16,1) = wh;

% Learning Rate:
gamma = 0.1;
% gamma = 1;
%  gamma = 0.02; % p_target(5,1) = 2*pi;
% gamma = 0.5;

% kappa = 1 for 2nd order dynamics expansion, kappa = 0 for 1st order
% dynamics expansion.
kappa = 0;
% kappa = 1;

% step length for finite differencing
deltax = 0.001; 

% Number of Iterations of Control Updates
% num_iter = 1;
num_iter = 60;
%  num_iter = 500;

% Time for Iteration 
tt = linspace(0, Tf, Horizon);
tt_u = tt(1:(Horizon - 1));

% Initial Control:          May need to set u(Tf) == 0 !!!
u = zeros(numControls,Horizon-1);
du = zeros(numControls,Horizon-1);
% 
% u(1,:) = -1e2*ones(1,Horizon - 1);
% u(2,:) = -1e2*ones(1,Horizon - 1);
% u(3,:) = -1e2*ones(1,Horizon - 1);
% u(4,:) = -1e2*ones(1,Horizon - 1);

% u(1,:) = 2e3*ones(1,Horizon - 1);

u_cont = spline(tt_u,u);

% Initial trajectory:
options= odeset('OutputFcn','');
 [T, x_traj] = ode45('QuadRotor', tt, xo, options, u_cont);
 
x_traj_cont = spline(tt, x_traj');
Cost = zeros(1, num_iter);
TerminalCost = zeros(1, num_iter);

for k = 1:num_iter
%------------------------------------------------> Calculate the cost

TerminalCost(:, k) = 0.5 * (x_traj(Horizon,:)' - p_target)'* Q_f * (x_traj(Horizon,:)' - p_target);

[T, RunningCost] = ode45('running_cost', tt, 0, options, u_cont, x_traj_cont);

% [T, RunningCost] = ode23t('running_cost', tt, 0, options, u_cont);

Cost(:,k) = RunningCost(end) + TerminalCost(:, k);

fprintf('Iteration %d,  Current Cost = %e \n',k,Cost(1,k));
   
%------------------------------------------------> Initial Condition of the
% Value function
Vxx_end= Q_f;
Vx_end = Q_f * (x_traj(end, :)' - p_target); 
V_end = 0.5 * (x_traj(end, :)' - p_target)' * Q_f * (x_traj(end, :)' - p_target); 

Vo = [V_end, Vx_end(:)', Vxx_end(:)'];

%------------------------------------------------> Integrate Backward the Value Function
tt_back = linspace(Tf, 0, Horizon);

[T, Vvalue] = ode45('ricattis', tt_back, Vo, options, u_cont, x_traj_cont);

% [T, Vvalue] = ode23t('ricattis', tt_back, Vo, options, u_cont, x_traj_cont);

% [T, Vvalue] = ode113('ricattis', tt_back, Vo, options, u_cont, x_traj_cont);

V = Vvalue(end:-1:1, 1);
Vx = Vvalue(end:-1:1, 2:numStates+1)';
Vxx = zeros(numStates, numStates, Horizon);
for i = 1:Horizon
    Vxx(1:numStates,1:numStates,i) = reshape(Vvalue(end+1-i, numStates+2:end), numStates, numStates);
end

V_cont = spline(tt, Vvalue(end:-1:1, :)');


%% 
%----------------------------------------------> Update the controls
Fu = G;
dxo = zeros(numStates, 1);
% du_cont = spline(tt_u,du);

[T, dx] = ode45('dxupdate_new', tt, dxo, options, u_cont, x_traj_cont, V_cont);

for i = 1:(Horizon-1)    
    lt(:, i) = - u(:, i) - R\Fu' * Vx(:, i);
    Lt(:, :, i) = - R\Fu' * Vxx(:, :, i);
    du(:, i) = lt(:, i) + Lt(:, :, i) * dx(i, :)';
    u_new(:, i) = u(:, i) + gamma*du(:, i);
end


u = u_new;

%%
%---------------------------------------------> Simulation of the Nonlinear System
u_cont = spline(tt_u,u);

 [T, x_traj] = ode45('QuadRotor', tt, xo, options, u_cont);
 
x_traj_cont = spline(tt, x_traj');
 
end

   time(1)=0;
   for i= 2:Horizon
    time(i) =time(i-1) + dt;  
   end

     
%% ---------------------------------------------> Plot Section

   figure()
   for i = 1:numStates
       subplot(4,4,i)
       hold on
       plot(tt,x_traj(:,i),'linewidth',4);
       plot(tt,p_target(i,1)*ones(1,Horizon),'--r','linewidth',4)
       title(['x', num2str(i)],'fontsize',20);
       xlabel('Time in sec','fontsize',20)
       hold off;
       grid;
   end
   
   figure()
   for i = 1:numControls
   subplot(2,2,i);hold on
   plot(time(1:end-1), u(i, :), 'linewidth',2);
   xlabel('Time in sec', 'fontsize',20)
   title(['Control', num2str(i)], 'fontsize',20)
   end
   
   figure() 
   hold on
   plot(Cost,'linewidth',2); 
   xlabel('Iterations','fontsize',20)
   title('Cost','fontsize',20);

   figure() 
   hold on
   plot(TerminalCost,'linewidth',2); 
   xlabel('Iterations','fontsize',20)
   title('Terminal Cost','fontsize',20);
   
