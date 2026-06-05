function [x_traj_ap] =  model1(t,xoo,flag,gprMdl1,gprMdl2,gprMdl3,gprMdl4,gprMdl5,gprMdl6,gprMdl7,gprMdl8,gprMdl9,gprMdl10,gprMdl11,gprMdl12,gprMdl13,gprMdl14,gprMdl15,gprMdl16,x_traj,u)



U=ppval(u, t);
in=[ppval(x_traj(1), t) ppval(x_traj(2), t) ppval(x_traj(3), t) ppval(x_traj(4), t) ppval(x_traj(5), t) ppval(x_traj(6), t) ppval(x_traj(7), t) ppval(x_traj(8), t) ppval(x_traj(9), t) ppval(x_traj(10), t) ppval(x_traj(11), t) ppval(x_traj(12), t) ppval(x_traj(13), t) ppval(x_traj(14), t) ppval(x_traj(15), t) ppval(x_traj(16), t) U'];
dxh1 = predict(gprMdl1,in);
dxh2 = predict(gprMdl2,in);
dxh3 = predict(gprMdl3,in);
dxh4 = predict(gprMdl4,in);
dxh5 = predict(gprMdl5,in);
dxh6 = predict(gprMdl6,in);
dxh7 = predict(gprMdl7,in);
dxh8 = predict(gprMdl8,in);
dxh9 = predict(gprMdl9,in);
dxh10 = predict(gprMdl10,in);
dxh11 = predict(gprMdl11,in);
dxh12 = predict(gprMdl12,in);
dxh13 = predict(gprMdl13,in);
dxh14 = predict(gprMdl14,in);
dxh15 = predict(gprMdl15,in);
dxh16 = predict(gprMdl16,in);


x_traj_ap=[dxh1;dxh2;dxh3;dxh4;dxh5;dxh6;dxh7;dxh8;dxh9;dxh10;dxh11;dxh12;dxh13;dxh14;dxh15;dxh16];