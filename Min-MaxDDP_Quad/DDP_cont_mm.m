function output = DDP_cont_mm(datain)

global Quadrotor

Q_f = datain.auxdata.Q_f;
Ru = datain.auxdata.Ru;
Rv = datain.auxdata.Rv;
Q = datain.auxdata.Q;
gamma = datain.gamma;
p_target = datain.auxdata.p_target;
kappa = datain.auxdata.kappa;
xo = datain.xo;
dxo = datain.dxo;
u_ref = datain.u_ref;
v_ref = datain.v_ref;
num_iter = datain.num_iter;
Tf = datain.Tf;
time = datain.time;
Ntime = datain.Ntime;
sysEOM = datain.EOMfile;
systemCOST = datain.COSTfile;

dim_x = size(xo,1);
dim_u = size(u_ref,1);
dim_v = size(v_ref,1);

abs_tol = [1e-5*ones(1,dim_x), 1e-2, 1e-5*ones(1,dim_x)];

% options = odeset('refine','1','OutputFcn','odeplot','RelTol',1e-4,'AbsTol',1e-5);
% options = odeset('refine','1','OutputFcn','','RelTol',1e-3,'AbsTol',1e-5);
options = odeset('refine','1','OutputFcn','','RelTol',1e-3,'AbsTol',abs_tol);

% Forward integration to generate nominal trajectory
[T,X_ref] = ode45('Xdot_Quadrotor_mm',time,[xo;0;dxo],options,u_ref,v_ref,[],[],time,Ru,Rv,Q,gamma,p_target);
x_ref = X_ref(:,1:dim_x)';

%--------------------------- start DDP iteration -----------------------

for k = 1:num_iter

Cost(:,k) =  systemCOST(X_ref(:,1:dim_x+1)',p_target,Q_f);
fprintf('Iteration %d,  Current Cost = %e \n',k,Cost(1,k));

% if (k>1) && (abs(Cost(1,k)-Cost(1,k-1)) < 1e-4)
%     disp('Iteration stopped owing to small cost improvement') 
%     break
% end

% Boundary conditions for V, Vx, Vxx (at Tf)
Vxx_f = Q_f;
Vx_f = Q_f*(x_ref(:,end)-p_target); 
V_f = 0.5*(x_ref(:,end)-p_target)'*Q_f*(x_ref(:,end)-p_target); 
Vstate_f = [V_f  Vx_f' Vxx_f(:)'];

% Compute control gains
% Involves backward integration of V, Vx, Vxx along u_ref and x_ref
V_ref = bw_value_func(time,Vstate_f,x_ref,u_ref,v_ref,Ru,Rv,Q,kappa,p_target);
% keyboard()

% Forward integration of trajectory
%options = odeset('refine','1','OutputFcn','odeplot','RelTol',1e-4,'AbsTol',1e-5);
% [T,X_ref] = ode15s('Xdot_Quadrotor_mm',time,[xo;0;dxo],options,u_ref,v_ref,x_ref,V_ref(end:-1:1,:),time,Ru,Rv,Q,gamma,p_target);
[T,X_ref] = ode45('Xdot_Quadrotor_mm',time,[xo;0;dxo],options,u_ref,v_ref,x_ref,V_ref(end:-1:1,:),time,Ru,Rv,Q,gamma,p_target);
x_ref = X_ref(:,1:dim_x)';
dx = X_ref(:,dim_x+2:2*dim_x+1)';


du = zeros(dim_u,length(time));
dv = zeros(dim_u,length(time));
Vx = V_ref(end:-1:1,2:1+dim_x)';
Vxx = zeros(dim_x,dim_x,Ntime);
for i = 1:length(T)
    for j = 1:dim_x
    Vxx(1:dim_x,j,i) = V_ref(end+1-i,(2+dim_x*j):(1+dim_x*(j+1)));
    end
end
for i=1:length(time)    
    u_i = u_ref(:,i);
    v_i = v_ref(:,i);
    %     x_i = x_ref(:,i);
%     [~,Fu,Fv] = ABCsys(x_i,u_i,v_i,time(i));   % one way to calculate Fu and Fv
    Fu = Quadrotor.G;
    Fv = Fu;
    Fuu = zeros(dim_u);
    Fvv = zeros(dim_v);
    Fux = zeros(dim_u,dim_x);
    Fxu = zeros(dim_x,dim_u);
    Fvx = zeros(dim_v,dim_x);
    Fxv = zeros(dim_x,dim_v);
    Fuv = zeros(dim_u,dim_v);
    Fvu = zeros(dim_v,dim_u);
    
    % pde of running cost L
    Lu = Ru*u_i;
    Lv = -Rv*v_i;
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
    
    lu(:, i) = -(Quu - Quv/Qvv*Qvu)\(Qu - Quv/Qvv*Qv);
    lv(:, i) = -(Qvv - Qvu/Quu*Quv)\(Qv - Qvu/Quu*Qu);
    Ku(:, :, i) = -(Quu - Quv/Qvv*Qvu)\(Qux - Quv/Qvv*Qvx);
    Kv(:, :, i) = -(Qvv - Qvu/Quu*Quv)\(Qvx - Qvu/Quu*Qux);
    
    du(:, i) = lu(:, i) + Ku(:, :, i) * dx(:, i);
    dv(:, i) = lv(:, i) + Kv(:, :, i) * dx(:, i);
end
  
u_ref = u_ref + gamma*du;
v_ref = v_ref + gamma*dv;

% keyboard()
  
end

% prepare output

output.state = x_ref;
output.time = time;
output.control_u = u_ref;
output.control_v = v_ref;
output.cost = Cost;
output.lu = lu;
output.lv = lv;
output.Ku = Ku;
output.Kv = Kv;
output.auxdata.p_target = p_target;

% forward path integration with more accurate ode
% u_ref(:,end) = zeros(dim_u,1);
% v_ref(:,end) = zeros(dim_v,1);
% [T,X_ref] = ode113('Xdot_TwoLink_mm',time,[xo;0;dxo],options,u_ref,v_ref,[],[],time,Ru,Rv,Q,gamma,p_target);
% output.state = X_ref(:,1:dim_x)';
% output.control_u = u_ref;
% output.control_v = v_ref;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function V_ref = bw_value_func(time,Vstate_f,x_traj,u_traj,v_traj,Ru,Rv,Q,kappa,p_target)
% returns value function and its derivatives in backward time order

options = odeset('refine','1','OutputFcn','','RelTol',1e-2,'AbsTol',1e-3);
% options = odeset('refine','1','OutputFcn','','RelTol',1e-3,'AbsTol',1e-4);
% Backward integration of V, Vx, Vxx
[Tback,Vstate] = ode45(@EOM_Value,flip(time),Vstate_f,options,u_traj,v_traj,x_traj,time,Ru,Rv,Q,kappa,p_target);
% T = flip(Tback);
V_ref = Vstate;


end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Vprime = EOM_Value(t,V,u_traj,v_traj,x_traj,time,Ru,Rv,Q,kappa,p_target)
% Vstate = [V, Vx(1), Vx(2), Vxx(1, 1), Vxx(1, 2), Vxx(2, 1), Vxx(2, 2)]

dim_x = size(x_traj,1);
dim_u = size(u_traj,1);
dim_v = size(v_traj,1);

x = interp1(time',x_traj',t)';
u = interp1(time(:,1:end)',u_traj(:,1:end)',t)';
v = interp1(time(:,1:end)',v_traj(:,1:end)',t)';
[Fx,Fu,Fv] = ABCsys(x,u,v,t);
Fuu = zeros(dim_u);
Fvv = zeros(dim_v);
Fux = zeros(dim_u,dim_x);
Fxu = zeros(dim_x,dim_u);
Fvx = zeros(dim_v,dim_x);
Fxv = zeros(dim_x,dim_v);
Fuv = zeros(dim_u,dim_v);
Fvu = zeros(dim_v,dim_u);

% pde of running cost L
L = (u'*Ru*u - v'*Rv*v + (x-p_target)'*Q*(x-p_target))/2;
Lx = Q*(x-p_target);
Lu = Ru*u;
Lv = -Rv*v;
Lxx = Q;
Luu = Ru;
Lvv = -Rv;
Lux = zeros(dim_u,dim_x);
Lxu = zeros(dim_x,dim_u);
Lvx = zeros(dim_v,dim_x);
Lxv = zeros(dim_x,dim_v);
Luv = zeros(dim_u,dim_v);
Lvu = zeros(dim_v,dim_u);

Vx = V(2:1+dim_x);
Vxx = reshape(V((2+dim_x):end), dim_x, dim_x);

% det_Vxx = det(Vxx);
% if det_Vxx <= 0
%     fprintf('Time %f,  det_Vxx = %e, Vxx = [%f %f; %f %f] \n',t, det_Vxx, Vxx(1,1), Vxx(1,2), Vxx(2,1), Vxx(2,2));
% end

Qx = Fx'*Vx + Lx;
Qu = Fu'*Vx + Lu;
Qv = Fv'*Vx + Lv;
Qxx = Lxx + 2*Vxx*Fx;
Quu = Luu + kappa*Fuu;
Qvv = Lvv + kappa*Fvv;
Qux = 1/2*(Lux + kappa*Fux) + 1/2*(Lxu + kappa*Fxu)' + Fu'*Vxx;
Qvx = 1/2*(Lvx + kappa*Fvx) + 1/2*(Lxv + kappa*Fxv)' + Fv'*Vxx;
Quv = 1/2*(Luv + kappa*Fuv) + 1/2*(Lvu + kappa*Fvu)';
Qvu = Quv';

lu = -(Quu - Quv/Qvv*Qvu)\(Qu - Quv/Qvv*Qv);
lv = -(Qvv - Qvu/Quu*Quv)\(Qv - Qvu/Quu*Qu);
Ku = -(Quu - Quv/Qvv*Qvu)\(Qux - Quv/Qvv*Qvx);
Kv = -(Qvv - Qvu/Quu*Quv)\(Qvx - Qvu/Quu*Qux);

dVdt = -(L + lu'*Qu + lv'*Qv + 0.5*lu'*Quu*lu + lu'*Quv*lv + 0.5*lv'*Qvv*lv);
dVxdt =-(Qx + Ku'*Qu + Kv'*Qv + Qux'*lu + Qvx'*lv + Ku'*Quu*lu + Kv'*Qvv*lv + Ku'*Quv*lv + Kv'*Qvu*lu);
dVxxdt =-(Qxx + Ku'*Quu*Ku + Kv'*Qvv*Kv + 2*Ku'*Qux + 2*Kv'*Qvx + 2*Ku'*Quv*Kv);

dVxxdt = dVxxdt/2 + dVxxdt'/2; % for symmetry

Vprime = [dVdt;  dVxdt(:);  dVxxdt(:)] ;

% keyboard()

end




             
   

             
   



