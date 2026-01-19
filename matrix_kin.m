function [J] = matrix_kin(q)
l = 0.3;
J11 = -l * sin(q(1)) - l * sin(q(1) + q(2));
J12 = -l * sin(q(1) + q(2));
J21 = -l*cos(q(1)) - l*cos(q(1) + q(2));
J22 = -l * cos(q(1) - q(2));

J = [J11 J12;...
     J21  J22];
