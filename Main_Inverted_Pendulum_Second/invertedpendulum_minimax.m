function xprime = invertedpendulum_minimax(t, x, flag, u, v)

global m1;
global grav;
global l1;
global I1;
global b1;

xprime = [ x(2); ...
    m1 * grav * l1 * sin(x(1))/I1 - b1*x(2)/I1 + ppval(u, t)/I1 - ppval(v, t)/I1];