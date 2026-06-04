function dxprime = dxupdate_minimax(t, dx, flag, u, v, x_traj, V)

global m1;
global grav;
global l1;
global I1;
global b1;
global Q_f;
global Ru;
global Rv;
global kappa;

x1_t = ppval(x_traj(1), t);
Vx = [ppval(V(2),t); ppval(V(3),t)];
Vxx = [ppval(V(4),t), ppval(V(6),t); ppval(V(5),t), ppval(V(7),t)];

dim_x = size(x_traj, 2);
dim_u = size(u, 2);
dim_v = size(v, 2);

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
L = (Ru*(ppval(u, t)^2) - Rv*(ppval(v, t)^2))/2;
Lx = zeros(dim_x,1);
Lu = Ru*ppval(u, t);
Lv = -Rv*ppval(v, t);
Lxx = zeros(dim_x);
Luu = Ru;
Lvv = -Rv;
Lux = zeros(dim_u,dim_x);
Lxu = zeros(dim_x,dim_u);
Lvx = zeros(dim_v,dim_x);
Lxv = zeros(dim_x,dim_v);
Luv = zeros(dim_u,dim_v);
Lvu = zeros(dim_v,dim_u);

Vx = [ppval(V(2),t); ppval(V(3),t)];
Vxx = [ppval(V(4),t) ,ppval(V(6),t); ...
    ppval(V(5),t) ,ppval(V(7),t)];

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

dxprime = [dx(2); ...
    m1 * grav * l1 * cos(x1_t)*dx(1)/I1 - b1*dx(2)/I1 + du/I1 - dv/I1 ];