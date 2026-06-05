function [Xdot] = Xdot_Quadrotor_mm(t,X,flag,u_traj,v_traj,x_traj,V_traj,time,Ru,Rv,Q,gamma,p_target)  
% forward integration of state, running cost and dx

kappa = 0;
dim_x = 0.5*(size(X,1)-1);
dim_u = size(u_traj,1);
dim_v = size(v_traj,1);
x = X(1:dim_x);
dx = X(dim_x+2:2*dim_x+1);

u_ref = interp1(time',u_traj',t)';
v_ref = interp1(time',v_traj',t)';
% u_ref = interp1(time(:,1:end-1)',u_traj(:,1:end-1)',t,'linear','extrap')';
if ~isempty(x_traj)
   x_ref = interp1(time',x_traj',t)';
end

%[A,B] = ABsys(x,ppval(u,t),t);

if ~isempty(V_traj)
    Vvalue = interp1(time', V_traj, t)';
    Vx = Vvalue(2:(1+dim_x));
    Vxx = reshape(Vvalue((2+dim_x):end), dim_x, dim_x);
    
    [A,B,C] = ABCsys(x_ref,u_ref,v_ref,t);
    
    % pde of dynamics F
    Fx = A;
    Fu = B;
    Fv = C;
    Fuu = zeros(dim_u);
    Fvv = zeros(dim_v);
    Fux = zeros(dim_u,dim_x);
    Fxu = zeros(dim_x,dim_u);
    Fvx = zeros(dim_v,dim_x);
    Fxv = zeros(dim_x,dim_v);
    Fuv = zeros(dim_u,dim_v);
    Fvu = zeros(dim_v,dim_u);
    
    % pde of running cost L
    Lu = Ru*u_ref;
    Lv = -Rv*v_ref;
    Luu = Ru;
    Lvv = -Rv;
    Lux = zeros(dim_u,dim_x);
    Lxu = zeros(dim_x,dim_u);
    Lvx = zeros(dim_v,dim_x);
    Lxv = zeros(dim_x,dim_v);
    Luv = zeros(dim_u,dim_v);
    Lvu = zeros(dim_v,dim_u);
    
    Qu = Fu'*Vx + Lu;
    Qv = Fv'*Vx + Lv;
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
    
    du = lu + Ku * dx;
    dv = lv + Kv * dx;
    dxdot = Fx*dx + Fu*du + Fv*dv;
else
    du = zeros(dim_u,1);
    dv = zeros(dim_v,1);
    dxdot = zeros(size(dx));
end

[Fx,G_x] = EOM(x,t);
U = u_ref + gamma*du;
V = v_ref + gamma*dv;
xdot = Fx + G_x*U + G_x*V;

%xdot = Fx + G_x*ppval(u,t);

Xdot = [xdot; 0.5*U'*Ru*U - 0.5*V'*Rv*V + 0.5*(x-p_target)'*Q*(x-p_target); dxdot];
      
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Fx,G_x] = EOM(x,t)

Fx = Fofx(x);
G_x = Gofx(x);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function F_x = Fofx(X)

global Quadrotor

m = Quadrotor.m;
grav = Quadrotor.grav;
L = Quadrotor.L;
I = Quadrotor.I;
k_F = Quadrotor.k_F;
k_m = Quadrotor.k_m;
k_M = Quadrotor.k_M;
wh = Quadrotor.wh;

phi = X(4:6);
omega = X(13:16);
F = k_F * omega.^2;
M = k_M * omega.^2;

T = [cos(phi(2)), 0, -cos(phi(1))*sin(phi(2)); ...
    0, 1, sin(phi(1)); ...
    sin(phi(2)), 0, cos(phi(1))*cos(phi(2))];

Rotation_matrix = [cos(phi(3))*cos(phi(2)) - sin(phi(1))*sin(phi(3))*sin(phi(2)), -cos(phi(1))*sin(phi(3)), cos(phi(3))*sin(phi(2)) + cos(phi(2))*sin(phi(1))*sin(phi(3)); ...
    sin(phi(3))*cos(phi(2)) + sin(phi(1))*cos(phi(3))*sin(phi(2)), cos(phi(1))*cos(phi(3)), sin(phi(3))*sin(phi(2)) - cos(phi(2))*sin(phi(1))*cos(phi(3)); ...
    -cos(phi(1))*sin(phi(2)), sin(phi(1)), cos(phi(1))*cos(phi(2))];

F_x = [X(7:9); ...
    T\X(10:12); ...
    [0, 0, -grav]' + Rotation_matrix * [0, 0, sum(F)/m]'; ...
    I\([L*(F(2) - F(4)), L*(F(3) - F(1)), M(1) - M(2) + M(3) - M(4)]' - cross(X(10:12), I*X(10:12))); ...
    k_m*(wh*ones(4,1) - omega)];
        
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Gx = Gofx(x)

global Quadrotor;

Gx = Quadrotor.G;
        
end

