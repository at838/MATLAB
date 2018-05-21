%---------------------------- Seesaw.ini ----------------------------%
%--------------------------------------------------------------------%
clear all
clc
%--------------------------------------------------------------------%
%---- Initial declarations for both linear and non-linear system ----%
%--------------------------------------------------------------------%

Jb = 0.5;                              % Moment of Inertia.
mf = 0.1;                              % Mass of the wagon.
h = 0.1;                               % Height of the wagon.
g = 9.81;                              % Gravitational acceleration.
Jconst = Jb + mf*h*h;                  % 

%--------------------------------------------------------------------%
%---- Matrices defined for linear system State Space Model ----------%
%--------------------------------------------------------------------%

A = [0 mf*g*h/Jconst 0 -mf*g/Jconst;1 0 0 0;0 -g 0 0;0 0 1 0];
B = [1/Jconst;0;0;0];
c = eye(4);
D = [0;0;0;0];

%--------------------------------------------------------------------%
%------------------- Pole placement at -1 ---------------------------%
%--------------------------------------------------------------------%

p0 = 1;
pol = [-p0,-p0,-p0,-p0];

%--------------------------------------------------------------------%
%--- Calculation of state feedback vector using ackermann formula ---%
%--------------------------------------------------------------------%

k=acker(A,B,pol);
C = [0 0 0 1];              % New C matrix for calculating pre-amplifier.
p = 1/(C*((B*k-A)^-1)*B);   % Pre-amplifier for simple state feedback.
sts = ss(A,B,c,D);
%--------------------------------------------------------------------%
%-------------------- PI calculations -------------------------------%
%--------------------------------------------------------------------%

A_hat =vertcat([A zeros(4,1)],[-C 0]); % to make A^ = |A  0| and B^= |B|
B_hat = vertcat(B,0);                  %              |-C 0|         |0|
pol1 = [-p0,-p0,-p0,-p0,-p0];          % Pole placement @ -1.
K_hat = acker(A_hat,B_hat,pol1);       % Ackermann for new K and p.
K2 = K_hat(1:4);                       %    K
PI = K_hat(5);                         % Proportional gain for PI. 
k2 = K2-p*C;                           % State feedback correction. 

%--------------------------------------------------------------------%
%----------------------- Zeitdiskrete -------------------------------% 
%--------------------------------------------------------------------%

Ts = 1/8000;                           % Sampling Time 
sysd =c2d(sts,Ts,'zoh');               % Continuous to Discrete conversion.
[Ad,Bd,Cd,Dd] = ssdata(sysd);          % Discrete State Space Matrices. 

%--------------------------------------------------------------------%
%------------------- Zustandsbeobachter -----------------------------%
%--------------------------------------------------------------------%

pp = 3;                                % Pole Placement @ -3
pol2 = [-pp,-pp,-pp,-pp];
ct = C.';
HT = acker(A.',ct,pol2);               % Ackermann for Observer
H = HT.';

%--------------------------------------------------------------------%
%------------------- Time Discrete Observer -------------------------%
%--------------------------------------------------------------------%

syms s;                                % Symbol 's' for equation.
expr = (s+pp)^4;                       % (s+3)^4.
den =expand (expr);                    % expand expression. 
tfden = sym2poly(den);                 
num = 1;
sysp = tf(num,tfden);                  % Transfer function 1/(s+pp)^4.
sysdp = c2d(sysp,Ts);                  % Continuous to discrete conversion.
[z,polt,kk] = zpkdata(sysdp);          % Poles extracted from discrete  
                                       % transfer function. 
pol = cell2mat(polt);                  % Convert cell to row matrix
Htd = acker(Ad.',ct,pol.');            % Ackermann
Hd = Htd.';

%--------------------------------------------------------------------%
%------------------------ END OF CODE -------------------------------%
%--------------------------------------------------------------------%

