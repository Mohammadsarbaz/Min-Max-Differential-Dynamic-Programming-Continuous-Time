function [Cost] =  fnCostComp_minimax(x_traj,u_new,v_new,p_target,dt)

global Q_f;
global Ru;
global Rv;


 [Horizon, numOfStates] = size(x_traj);
 Cost = 0;
 
 for j =1:(Horizon-1)
     
    Cost = Cost + 0.5 * u_new(:,j)' * Ru * u_new(:,j) * dt - 0.5 * v_new(:,j)' * Rv * v_new(:,j) * dt;
     
 end
 
 TerminalCost= 0.5*(x_traj(Horizon,:)' - p_target)'*Q_f * (x_traj(Horizon,:)' - p_target);
 
 Cost = Cost + TerminalCost;