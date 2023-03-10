%% Ceci est le fichier du BDE de Commande robuste année 2022/2023
% Colaborateurs : 
% Emilien Reuillard Et Antonin Renoir

clear all
close all
% Pour chaque question voir le compte rendu pour les explications. 

%% Pensez à bien compiler le fichier Matlab

%% Question 1.1: Flight dynamics (10%)

% Rédigé sur le compte rendu. 

%% Question 1.2: Model construction & analysis (10%)
%Constantes
g = 9.80665;
a = 316.0561;
M = 3;
V = M*a; 
Zalpha = 1236.8918;
Malpha = -300.4211;
Mq = 0;
Zdelta = 108.1144;
Mdelta = -131.3944;
Aalpha = 1434.7783;
Adelta = 115.0529;
wa = 150;
ZetaAlpha = 0.7;

%%% définition des variables
g = 9.80665; %standard gravity acceleration
a = 316.0561; %speed of sound at 6096m
M = 3; %Mach number
Zalpha = 1236.8918; %normal force derivative
Malpha = -300.4211; %Pitch moment derivative
Mq = 0; %damping derivative
Zdelta = 108.1144; %control force derivative
Mdelta = -131.3944; %control moment derivative
Aalpha = 1434.7783; %normal acceleration derivative
Adelta = 115.0529; %control acceleration derivative
Walpha = 150; %actuator natural frequency
Zeta = 0.7; %actuator damping ratio
V = M*a; %airspeed

s=tf("s");

%equations d'état
%linear model

A1 = [[-Zalpha/V, 1]
      [Malpha, Mq]];

B1 = [ -Zdelta/V; 
        Mdelta ];

C1 = [[-Aalpha/g, 0]
      [0, 1]];

D1 = [-Adelta/g;
      0];

G_m = ss(A1,B1,C1,D1);

G_m.InputName="u_m";
G_m.StateName=["x1","x2"];
G_m.OutputName=["y1","y2"];
save G_m;

%Actuator

A2 = [  [0, 1]
        [-(wa*wa), -2*ZetaAlpha*wa]];

B2 = [  0;
        (wa*wa) ];

C2 = [  [1 0]
        [0 1]  ];

D2 = [0 ; 0];

G_a = ss(A2,B2,C2,D2);

G_a.InputName="u_cmd";
G_a.StateName=["x3","x4"];
G_a.OutputName=["u_m","udot_m"];
save G_a;


%linearisation
G_am = linearize('Airframe');  
G_am_zpk = zpk(G_am); 
%iopzmap(G_am); %Affichage des zeros


%% Question 2.1: Damping gain design (5%)

G_ol_q = -G_am(2,1); 
%rlocus(G_ol_q);
%avec rlocus on a que : C_q = -0.16;
C_q = -0.16;
G_cl_q_unsc = linearize("ClosedLoop_Cq");
G_cl_q_unsc.InputName="u_unsc";
zpk(G_cl_q_unsc);
zpk(G_am(1,1));

%% Question 2.2: Scaling gain design (5%)

C_sc_inv = dcgain(G_cl_q_unsc);
C_sc = 1/C_sc_inv;
G=linearize("ClosedLoop_CqCsc");
%zpk(G);
%step(G);

%% Question 2.3: Integral gain design (10%)

% C_i = 1;   %arbitraire pour commencer 
C_i = 8.0167e+05/C_sc;
G_ol_nz = linearize("ClosedLoop_CqCscCi");
zpk(G_ol_nz);
C_i_sc = C_i*C_sc*(1/s);
%sisotool( G_ol_nz, 1, C_q, C_i_sc )


%% 3.	Mixed sensitivity design (60%)

%% A.	Weighting filters (5%)

%% Question 3A.1: Weighting filter discussion (2.5%)

%Voir compte rendu. 

%% Question 3A.2: Weighting filter computation (2.5%)

%W1 et W2

%Passe haut
DC1 = 0.1;
Freq1 = 4;
Mag1 = 3.01;   %dB
HF1 = 100;      %rad/s 
W1 = makeweight(DC1, [Freq1, Mag1], HF1);

%passe bas
DC2 = 100;
Freq2 = 4;
Mag2 = 15;   %dB
HF2 = 0.1;   %rad/s
W2 = makeweight(DC2, [Freq2, Mag2], HF2);

W3 = W1;

% Affichage
% bodemag(W1,W2)
% legend
% grid on

% zpk(W1)
% zpk(W2)

% On en déduit que:

M_1 = 1/100;
w_1 = 13.29;
A_1 = 132.9/w_1;

M_2 = 10;
w_2 = 6.069;
A_2 = w_2/6069;

% sigma(W1,W2)

M_1 = A_2;
A_1 = M_2;

%% Question 3B.1: Reference model computation (5%)
%close all 

tau = 0.6;
w_d = 21;
xi_d = 0.8;
z_m =500;
T_d = tf([-w_d*w_d*(1/z_m), w_d*w_d],[1, 2*xi_d*w_d, w_d*w_d]);

%step(T_d);

%% C.	Feedback controller design (hinfsyn) (20%)

%% Question 3C.1: Controller design (10%)

F_f = 1;
C_e = C_i*tf([0 1],[1 0]); %Ci * 1/s
S0 = inv(1 + G*C_e);
T0 = 1 - S0;

P = [   [W1     ,   -W1*G  ]
        [0      ,   W2     ]
        [W3*T_d  ,   -W3*G  ]
        [1      ,   -G     ]  ];

Twz = [W1*S0 ; W2*C_e*S0 ; W3*(T_d-T0)];


%% Fin
% Pour chaque question voir le compte rendu pour les explications. 










