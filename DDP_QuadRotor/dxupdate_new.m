function dxprime = dxupdate_new(t, dx, flag, u, x_traj, V)

global m grav k_F k_M k_m wh L I invI wmax wmin G numStates numControls R kappa deltax

X = ppval(x_traj, t);
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

X_prime = [X(7:9); ...
    T\X(10:12); ...
    [0, 0, grav]' + Rotation_matrix * [0, 0, sum(F)/m]'; ...
    I\([L*(F(2) - F(4)), L*(F(3) - F(1)), M(1) - M(2) + M(3) - M(4)]' - cross(X(10:12), I*X(10:12))); ...
    k_m*(wh*ones(4,1) - omega)];

dxs = zeros(numStates,1,numStates);
B_dx = zeros(numStates);

for i = 1:numStates
    dx_vec = zeros(numStates,1);
    dx_vec(i,1) = deltax;
    dxs(:,:,i) = dx_vec;
    B_dx(i,:) = dx_vec';
end

% B_dx_inv = inv(B_dx'*B_dx);
B_dx_inv = eye(numStates)/(deltax^2);

A_dx = zeros(numStates);
for i = 1:numStates
    X_new = X + dxs(:,:,i);
    phi = X_new(4:6);
    omega = X_new(13:16);
    F = k_F * omega.^2;
    M = k_M * omega.^2;
    
    T = [cos(phi(2)), 0, -cos(phi(1))*sin(phi(2)); ...
        0, 1, sin(phi(1)); ...
        sin(phi(2)), 0, cos(phi(1))*cos(phi(2))];
    
    Rotation_matrix = [cos(phi(3))*cos(phi(2)) - sin(phi(1))*sin(phi(3))*sin(phi(2)), -cos(phi(1))*sin(phi(3)), cos(phi(3))*sin(phi(2)) + cos(phi(2))*sin(phi(1))*sin(phi(3)); ...
        sin(phi(3))*cos(phi(2)) + sin(phi(1))*cos(phi(3))*sin(phi(2)), cos(phi(1))*cos(phi(3)), sin(phi(3))*sin(phi(2)) - cos(phi(2))*sin(phi(1))*cos(phi(3)); ...
        -cos(phi(1))*sin(phi(2)), sin(phi(1)), cos(phi(1))*cos(phi(2))];
    
    X_new_prime = [X_new(7:9); ...
        T\X_new(10:12); ...
        [0, 0, grav]' + Rotation_matrix * [0, 0, sum(F)/m]'; ...
        I\([L*(F(2) - F(4)), L*(F(3) - F(1)), M(1) - M(2) + M(3) - M(4)]' - cross(X_new(10:12), I*X_new(10:12))); ...
        k_m*(wh*ones(4,1) - omega)];
    A_dx(:,i) = X_new_prime - X_prime;
    
end

A_dx = A_dx';
Fx = zeros(numStates);

for i = 1:numStates
    Fx(i,:) = (B_dx_inv*B_dx'*A_dx(:,i))';
end

Fu = G;

v = ppval(V, t);
Vx = v(2:numStates+1);
Vxx = reshape(v(numStates+2:end), numStates, numStates);

% q0_t = ppval(u, t)'*R*ppval(u, t)/2;
% q_t = Q*(ppval(x_traj, t) - p_target);
r_t = R*ppval(u, t);
% Q_t = Q;
R_t = R;
N_t = zeros(numStates, numControls);
M_t = zeros(numStates, numControls)';

l_t = -R_t\(r_t + Fu'*Vx);
L_t = -R_t\(N_t'/2 + M_t/2 + Fu'*Vxx);


du = l_t + L_t * dx;

dxprime = Fx*dx + G*du;