function dx = int_2DoF(t,x,qref,qdref,tgrid,...
                        l1,l2,m1,m2,I1,I2,g,Kp,Kd,Fext,Fstart,Fend)
% Dinamika 2-link manipulatora
q  = x(1:2); qd = x(3:4);

% Interpolacija reference
qr   = interp1(tgrid,qref',t)';
qrd  = interp1(tgrid,qdref',t)';
[H,C,G]=matrix_dyn(l1,l2,m1,m2,I1,I2,g,qr,qrd);

% Spoljašnja sila -> u torzije
if (t>=Fstart && t<=Fend)
    J=matrix_kin(q);
    tau_dist = J'*Fext;
else
    tau_dist = [0;0];
end
tau = Kp.*(qr-q) + Kd.*(qrd-qd);

% Dinamičke jednačine
qdd = H \ (tau - C*qd - G + tau_dist);
dx = [qd; qdd];
end