

clear; clc; close all;
index = 217+115; 
%% 2) Parametri robota
l1 = 0.3; l2 = 0.3;        % dužine segmenata [m]
m1 = 2;  m2 = 2;           % mase segmenata [kg]
g  = 9.81;                 % gravitacija [m/s^2]
I1 = m1*l1^2/3;           % inercija homogenih šipki
I2 = m2*l2^2/3;

%% 3) Izračun T i tip upravljanja
T = 3 + mod(index,4);          % trajanje simulacije
% Trajanje faza
T1 = 0.5*T;   % A→B
T2 = 0.5*T;   % B→C
% Interval sile
F_start = 0.35*T;
F_end   = 0.40*T;

%% 4) Početne i ciljane konfiguracije
% Tačke u prostoru
A_q = deg2rad([-45;-45]);      % početna konfiguracija
B = [0.3; 0];           % B (x,z)
C = [0.3*sqrt(3); 0];   % C (x,z)

% Inverzna kinematika
B_q = inverzna(B,l1,l2);
C_q = inverzna(C,l1,l2);

disp('Konfiguracije (u stepenima):');
disp(['A = (',num2str(rad2deg(A_q(1))),', ',num2str(rad2deg(A_q(2))),')']);
disp(['B = (',num2str(rad2deg(B_q(1))),', ',num2str(rad2deg(B_q(2))),')']);
disp(['C = (',num2str(rad2deg(C_q(1))),', ',num2str(rad2deg(C_q(2))),')']);

%% 5) Generisanje referentnih trajektorija
dt = 0.01; tspan = 0:dt:T;
[qref, qdref] = planiranje_t(A_q,B_q,C_q,T1,T2,dt);

%% 6) Simulacija dinamike sa upravljanjem
x0 = [A_q; 0; 0];  % početno stanje (položaj + brzina)
PID_parameters;

% Spoljašnja sila u XY (X = horizontalno, Z = vertikalno)
Fext = [-2; 0];  % konst. sila duž -X

% ODE simulacija
[t,y] = ode45(@(t,x) int_2DoF(t,x,qref,qdref,tspan,l1,l2,m1,m2,I1,I2,g,Kp,Kd,Fext,F_start,F_end),tspan, x0);

q  = y(:,1:2);   % realni položaji
qd = y(:,3:4);   % realne brzine

%% Simulacija dinamike

N = length(tspan);
Ps = struct();     % inicijalizacija

for i = 1:N
    t_now = tspan(i);
    
    % trenutno stanje iz ODE simulacije
    x_now  = y(i,:)';
    q_now  = x_now(1:2);
    dq_now = x_now(3:4);
    
    % referentne vrednosti
    q_ref_now  = interp1(tspan', qref', t_now)';
    dq_ref_now = interp1(tspan', qdref', t_now)';
    ddq_ref_now = [0;0];  % ako nema ubrzanja
    
    % matrica inercije i dinamički parametri
    q = q_now; qd = dq_now;
    
    [H,C,G]=matrix_dyn(l1,l2,m1,m2,I1,I2,g,q,qd);
     
    % spoljašnja sila -> torzija
    if t_now >= F_start && t_now <= F_end
        J = matrix_kin(q);       % Jacobian
        Tau_now = J'*Fext;       % torzija od spoljašnje sile
        Fint_now = Fext;         
    else
        Tau_now = [0;0];
        Fint_now = [0;0];
    end
    
    % PID kontroler (decentralizovano)
    Tau_pid = Kp.*(q_ref_now - q_now) + Kd.*(dq_ref_now - dq_now);
    
    % Ukupni moment
    Tau_total = Tau_pid + Tau_now;
    
    % trenutna pozicija
    X_now = forward_kinematics(q_now);
    X_ref = forward_kinematics(q_ref_now);
    
    % čuvanje u memoriji
    Ps = write_in_memory(Ps, i, dt, q_now, dq_now, X_now, X_ref, ...
                         q_ref_now, dq_ref_now, ddq_ref_now, Tau_total, Fint_now);
end

%% Generisanje glatke putanje (ugao trećeg stepena) između A→B→C
q1 = [A_q(1), B_q(1), C_q(1)];
q2 = [A_q(2), B_q(2), C_q(2)];

% kreiramo interpolaciju
q = [interp1([0 T/2 T], q1, t, 'pchip');  % 1xN
     interp1([0 T/2 T], q2, t, 'pchip')]; % 1xN


%% Računanje pozicija A, B, C u radnom prostoru
A = forward_kinematics(A_q);
B = forward_kinematics(B_q);
C = forward_kinematics(C_q);

%% Crtanje
figure;
axis equal; grid on;
axis([-0.7 0.7 -0.7 0.7]);
xlabel('x [m]'); ylabel('z [m]');
title('2DoF robot - animacija');
hold all;

% Označi ciljne tačke
plot(A(1), -A(2), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'A'); hold all;
plot(B(1), B(2), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'B');
plot(C(1), C(2), 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'C');
legend show;

% Inicijalni izgled robota
hLine = plot([0,0,0],[0,0,0],'--ks','LineWidth',2,'MarkerFaceColor','y','MarkerSize',10);

% Animacija
%q = [interp1([0 T/2 T], q1, t, 'pchip'), interp1([0 T/2 T], q2, t, 'pchip')];
for i = 1:length(t)
    q1 = q(i,1);
    q2 = q(301+i,1);
    % koordinate zglobova
    x_joint1 = l1*cos(q1);
    z_joint1 = l1*sin(q1);
    x_end = x_joint1 + l2*cos(q1+q2);
    z_end = z_joint1 + l2*sin(q1+q2);

    % ažuriranje prikaza
    set(hLine,'XData',[0 x_joint1 x_end],'YData',[0 z_joint1 z_end]);
    drawnow;
end

%%
display_results
%%


%% --- LOKALNE FUNKCIJE -------------------------------------

function q = inverzna(pos,l1,l2)
% Inverzna kinematika 2-link planar (uzima "elbow-down" rešenje)
x=pos(1); z=pos(2);
c2 = (x^2+z^2 - l1^2 - l2^2)/(2*l1*l2);
s2 = sqrt(1-c2^2);
q2 = atan2(s2,c2);
q1 = atan2(z,x) - atan2(l2*sin(q2),l1+l2*cos(q2));
q = [q1; q2];
end

function [qref,qdref] = planiranje_t(A_q,B_q,C_q,T1,T2,dt)
% Generiše reference (cubic poly) za A→B i B→C
tspan1 = 0:dt:T1; tspan2 = T1:dt:T2;
[qAB,qdAB,qddAB] = cubic(A_q,B_q,T1,tspan1);
[qBC, qdBC] = PTP_step(dt, T2, B_q, C_q);

qref   = [qAB qBC];
qdref  = [qdAB qdBC];
end



