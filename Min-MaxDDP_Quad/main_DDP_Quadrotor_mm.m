clear all;  close all; format compact

parameters;

dim_x = 16;         % dimention of states
dim_u = 4;         % dimention of minimizing control inputs
dim_v = 4;         % dimention of maximizing control inputs

% Final Time
% Tf = 1.5; % pitch 2pi
Tf = 2; % pitch 2pi
% Tf = 5.0; % travel

% Weight in Final State (pitch 2pi)
Q_f = zeros(dim_x);
for i = 1:3
Q_f(i, i) = 1e7; 
end
for i = 4:6
Q_f(i, i) = 1e6; 
end
for i = 7:9
Q_f(i, i) = 1e6; 
end
for i = 10:12
Q_f(i, i) = 1e5; 
end
Q_f(5, 5) = 1e7;


% % Weight in Final State (travel)
% Q_f = zeros(dim_x);
% for i = 1:3
% Q_f(i, i) = 1e6; 
% end
% for i = 4:6
% Q_f(i, i) = 1e5; 
% end
% for i = 7:9
% Q_f(i, i) = 1e4; 
% end
% for i = 10:12
% Q_f(i, i) = 1e4; 
% end

% Weight in the State:
Q = 1e-2*Q_f;
% Q = 0*Q_f;

% Weight in the Controls:
% Ru = 0.0001*eye(dim_u);
% Rv = 0.001*eye(dim_v);

Ru = 0.0001*eye(dim_u);
Rv = 0.0005*eye(dim_v);
% Rv = 0.00005*eye(dim_v);

% Ru = 0.001*eye(dim_u);
% Rv = 0.01*eye(dim_v);

% Initial Configuration
xo = zeros(16,1);
% xo(7,1) = 5;
% xo(9,1) = 5;
% xo(12,1) = 0.5;
xo(13,1) = Quadrotor.wh;
xo(14,1) = Quadrotor.wh;
xo(15,1) = Quadrotor.wh;
xo(16,1) = Quadrotor.wh;
dxo = zeros(dim_x,1);

% Target: % position target
p_target(1,1) = 0;
p_target(2,1) = 0;
p_target(3,1) = 0;

% p_target(1,1) = 5;
% p_target(2,1) = 5;
% p_target(3,1) = 1;

% p_target(1,1) = 3;
% p_target(2,1) = 3;
% p_target(3,1) = 0;

% angular target
p_target(4,1) = 0;  % roll
p_target(5,1) = 2*pi;  % pitch
p_target(6,1) = 0;  % yaw

% p_target(4,1) = 0;  % roll
% p_target(5,1) = 0;  % pitch
% p_target(6,1) = pi;  % yaw

% p_target(4,1) = 0;  % roll
% p_target(5,1) = 2*pi;  % pitch
% p_target(6,1) = pi;  % yaw

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

% Learning Rate:
gamma = 0.5; % pitch 2pi
% gamma = 0.2;
% gamma = 0.15; % case 2
% gamma = 0.1; % travel

% kappa = 1 for 2nd order dynamics expansion, kappa = 0 for 1st order
% dynamics expansion.
% kappa = 1;
kappa = 0;

% Time for Iteration 
% Ntime = 101;
Ntime = 501;
time = linspace(0,Tf,Ntime);

% Initial Controls   
u_ref = zeros(dim_u,Ntime);
v_ref = zeros(dim_v,Ntime);

% Initial State  
% x_ref = zeros(dim_x,Ntime);

% Number of Iterations of Control Updates
% num_iter = 50; 
num_iter = 80; % travel

%------------------------------------------------------------------
%                  Collect Data
%------------------------------------------------------------------

datain.gamma = gamma;
datain.auxdata.p_target = p_target;
datain.auxdata.kappa = kappa;
datain.xo = xo;
datain.dxo = dxo;
datain.u_ref = u_ref;
datain.v_ref = v_ref;
datain.auxdata.Ru = Ru;
datain.auxdata.Rv = Rv;
datain.auxdata.Q = Q;
datain.auxdata.Q_f = Q_f;
datain.num_iter = num_iter;
datain.Tf = Tf;
datain.time = time;
datain.Ntime = Ntime;
datain.EOMfile = @Xdot_Quadrotor_mm;
datain.COSTfile = @Cost_mm;

%------------------------------------------------------------------
%                  Call Continuous DDP 
%------------------------------------------------------------------
tic;
sol = DDP_cont_mm(datain);
toc;

%%
%------------------------------------------------------------------
%                   Plot resuts
%------------------------------------------------------------------


MovieFile = PlotResults(sol,1,0);
% 
% if ~isempty(MovieFile)
% %    writerObj = VideoWriter([char(39),['Movie_',datestr(now,'yyyy-mm-dd_T_HH_MM_SS')],char(39)]);
%     writerObj = VideoWriter(['Movie_',datestr(now,'yyyy-mm-dd_T_HH_MM_SS')]);
%     open(writerObj);
%     writeVideo(writerObj,MovieFile);
%     close(writerObj);
% end