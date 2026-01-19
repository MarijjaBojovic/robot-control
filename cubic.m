function [q,qd,qdd] = cubic(q0,qf,tf,t)
tau = t/tf;
q = q0 + (qf-q0).*(3*tau.^2 - 2*tau.^3);
qd = (qf-q0).*(6*tau - 6*tau.^2)/tf;
qdd= (qf-q0).*(6 - 12*tau)/(tf^2);
end