function xprime = running_cost(t, x, flag, u, x_traj)

global R Q p_target

xprime = 0.5 * ( (ppval(x_traj, t) - p_target)' * Q * (ppval(x_traj, t) - p_target) + ppval(u, t)' * R * ppval(u, t));