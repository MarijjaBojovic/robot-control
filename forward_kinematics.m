function [X] = forward_kinematics(q)

l=0.3;
% Dekartove kordinate
x = l * cos(q(1)) + l * cos(q(1) + q(2));%  X
z = -l * sin(q(1)) - l * sin(q(1) + q(2));%  Z
X = [x; z];

