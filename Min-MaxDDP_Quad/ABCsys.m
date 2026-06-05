
function [A,B,C] = ABCsys(x,uref,vref,t)
    
global Quadrotor;

G = Quadrotor.G;

dim_x = size(x,1);
dim_u = size(uref,1);
dim_v = size(vref,1);

A = zeros(dim_x);
B = zeros(dim_x,dim_u);
C = zeros(dim_x,dim_v);

TOL = 1e-8;% Can change this to make numjac more/less accurate
y = QuadRotor4numjac(t,x);
A = numjac(@QuadRotor4numjac,0,x,y,TOL*ones(dim_x,1),[],0);

B = G;

C = B;

end
