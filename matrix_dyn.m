function [H,C,G] = matrix_dyn(l1,l2,m1,m2,I1,I2,g,q,qd)

% Matrica inercije 
H11 = I1+I2 + m1*(l1^2)/4 + m2*(l1^2+l2^2/4+l1*l2*cos(q(2)));
H12 = I2 + m2*(l2^2/4 + l1*l2*cos(q(2))/2);
H21 = H12;
H22 = I2 + m2*l2^2/4;
H = [H11 H12; H21 H22];

% Centr. i Coriolis
h = -m2*l1*l2*sin(q(2))/2;
C = [h*qd(2), h*(qd(1)+qd(2));
    -h*qd(1), 0];

% Gravitacioni vektor (z = vertikala)
G = [(m1*l1/2 + m2*l1)*g*cos(q(1)) + m2*l2/2*g*cos(q(1)+q(2));
      m2*l2/2*g*cos(q(1)+q(2))];

end