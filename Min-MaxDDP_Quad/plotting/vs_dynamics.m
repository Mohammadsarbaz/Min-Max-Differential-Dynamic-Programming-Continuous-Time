function dotS = vs_dynamics(t, S)
    
    global Sd
    n = size(S,1);
    
    num_points = n/3-1;
    
    x = S(1:num_points);
    y = S(num_points+1:2*num_points);
    Z = S(2*num_points+1:3*num_points);
    
    Psi = S(n-2);
    Theta = S(n-1);
    Phi = S(n);
    
    
    LS = zeros(2*num_points,6);
    LZ = zeros(num_points,6);
    for i = 1:num_points
        LS(2*i-1,:) = [-1/Z(i)   0   x(i)/Z(i)  x(i)*y(i)  -(1+x(i)^2)  y(i)];
        LS(2*i,:) = [ 0  -1/Z(i)  y(i)/Z(i) 1+y(i)^2    -x(i)*y(i)  -x(i)];
        LZ(i,:) = [0 0 -1 -y(i)*Z(i) x(i)*Z(i) 0];
    end
    
   
    lamda = 1;
    pseudo_invLS = LS'*inv(LS*LS');
    
    e = Sd - S(1:2*num_points);
    norm(e)
    vc = lamda*pseudo_invLS*e;
    
    
    
    Hinv = [0           sin(Phi) cos(Phi);
            0           cos(Theta)*cos(Phi) -cos(Theta)*sin(Phi);
            cos(Theta)  sin(Theta)*sin(Phi) sin(Theta)*cos(Theta)];
    
    dotS = [LS; ...
            LZ; ...
            zeros(3) 1/cos(Theta)*Hinv]*vc;    

end