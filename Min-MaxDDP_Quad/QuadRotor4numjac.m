function Xprime = QuadRotor4numjac(t, X)
% X = [x, y, z, phi (phi1), theta (phi2), psi (phi3), dot{x}, dot{y}, dot{z}, p, q, r, omega1, omega2, omega3, omega4]

global Quadrotor

m = Quadrotor.m;       % mass in kg
grav = Quadrotor.grav;    % gravitational acceleration in m/s^2
k_F = Quadrotor.k_F;   % force motor constant in N/(r/min^2)
k_M = Quadrotor.k_M;    % moment motor constant in Nm/(r/min^2)
k_m = Quadrotor.k_m;
wh = Quadrotor.wh;   % rotor speed to hold hover in r/min
L = Quadrotor.L;       % length of moment arm to rotors in m
Ixx = Quadrotor.Ixx;
Iyy = Quadrotor.Iyy;
Izz = Quadrotor.Izz;
I = [Ixx 0 0;0 Iyy 0; 0 0 Izz]; % mass moment of inertia matrix
invI = inv(I);  % inverse of mass moment of inertia matrix
wmax = Quadrotor.wmax;    % r/min
wmin = Quadrotor.wmin;    % r/min
phi = X(4:6);
omega = X(13:16);
F = k_F * omega.^2;
M = k_M * omega.^2;

Xprime(1:3) = X(7:9);
Xprime(4:6) = [cos(phi(2)), 0, -cos(phi(1))*sin(phi(2)); ...
    0, 1, sin(phi(1)); ...
    sin(phi(2)), 0, cos(phi(1))*cos(phi(2))]\X(10:12);
Xprime(7:9) = [0, 0, -grav]' + [cos(phi(3))*cos(phi(2)) - sin(phi(1))*sin(phi(3))*sin(phi(2)), -cos(phi(1))*sin(phi(3)), cos(phi(3))*sin(phi(2)) + cos(phi(2))*sin(phi(1))*sin(phi(3)); ...
    sin(phi(3))*cos(phi(2)) + sin(phi(1))*cos(phi(3))*sin(phi(2)), cos(phi(1))*cos(phi(3)), sin(phi(3))*sin(phi(2)) - cos(phi(2))*sin(phi(1))*cos(phi(3)); ...
    -cos(phi(1))*sin(phi(2)), sin(phi(1)), cos(phi(1))*cos(phi(2))] * [0, 0, sum(F)/m]';
Xprime(10:12) = I\([L*(F(2) - F(4)), L*(F(3) - F(1)), M(1) - M(2) + M(3) - M(4)]' - cross(X(10:12), I*X(10:12)));
Xprime(13:16) = k_m*(wh*ones(4,1) - omega);