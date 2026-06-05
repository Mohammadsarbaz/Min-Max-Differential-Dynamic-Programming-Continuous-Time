function Cost =  Cost_mm(x,xf,Qf)
% Cost calculation, where x(1:Nx,:) represents the trajectory and
% x(Nx+1,end) represents the integrated running cost
Nx = size(xf,1);
 
TerminalCost= 0.5*(x(1:Nx,end)-xf)'*Qf*(x(1:Nx,end)-xf);
Cost = TerminalCost + x(Nx+1,end);
 
end