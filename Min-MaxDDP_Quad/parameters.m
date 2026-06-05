global Quadrotor

% System Data


%%%%% Quadrotor Physical Properties
Quadrotor.m = 0.5;       % mass in kg
Quadrotor.grav = 9.81;    % gravitational acceleration in m/s^2
Quadrotor.k_F = 6.11E-8;   % force motor constant in N/(r/min^2)
Quadrotor.k_M = 1.5E-9;    % moment motor constant in Nm/(r/min^2)
Quadrotor.k_m = 1/20;
Quadrotor.wh = ((Quadrotor.m*Quadrotor.grav/4)/Quadrotor.k_F)^0.5;   % rotor speed to hold hover in r/min
Quadrotor.L = 0.38;       % length of moment arm to rotors in m
Quadrotor.Ixx = 3.9E-3;
Quadrotor.Iyy = 4.4E-3;
Quadrotor.Izz = 4.9E-3;
Quadrotor.I = [Quadrotor.Ixx 0 0;0 Quadrotor.Iyy 0; 0 0 Quadrotor.Izz]; % mass moment of inertia matrix
Quadrotor.invI = inv(Quadrotor.I);  % inverse of mass moment of inertia matrix
Quadrotor.wmax = 7800;    % r/min
Quadrotor.wmin = 1200;    % r/min
Quadrotor.G = zeros(16,4);
Quadrotor.G(13:16, :) = Quadrotor.k_m.*[1 0 -1 1; ...
    1 1 0 -1; ...
    1 0 1 1; ...
    1 -1 0 -1]; 