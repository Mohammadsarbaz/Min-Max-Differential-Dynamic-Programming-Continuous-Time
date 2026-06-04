function R = rot(Phi, Theta, Psi)

phi = [Phi, Theta, Psi];
R = [cos(phi(3))*cos(phi(2)) - sin(phi(1))*sin(phi(3))*sin(phi(2)), -cos(phi(1))*sin(phi(3)), cos(phi(3))*sin(phi(2)) + cos(phi(2))*sin(phi(1))*sin(phi(3)); ...
    sin(phi(3))*cos(phi(2)) + sin(phi(1))*cos(phi(3))*sin(phi(2)), cos(phi(1))*cos(phi(3)), sin(phi(3))*sin(phi(2)) - cos(phi(2))*sin(phi(1))*cos(phi(3)); ...
    -cos(phi(1))*sin(phi(2)), sin(phi(1)), cos(phi(1))*cos(phi(2))];

end