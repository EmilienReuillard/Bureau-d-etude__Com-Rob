%Ceci est le fichier du BDE de Commande robuste 
% Colaborateurs : Emilien Reuillard Et Antonin Renoir

%% Question 1.1: Flight dynamics (10%)

% Bien rédiger

%% Question 1.2: Model construction & analysis (10%)
%Constantes
g = 9.81;
a = 316.0561;
M = 3;
V = M; 
Zalpha = 1236.8918;
Malpha = -300.4211;
Mq = 0;
Zdelta = 108.1144;
Mdelta = -131.3944;
Aalpha = 1434.7783;
Adelta = 115.0529;
wa = 150;
ZetaAlpha = 0.7;

%equations d'état
%linear model
% X = (x1 x2) = (aplha q) et Y = (y1 y2) = (nz q)

A1 = [[-Zalpha/V, 1]
      [Malpha, Mq]];

B1 = [ -Zalpha/V; 
        Mdelta ];

C1 = [[-Aalpha/g, 0]
      [0, 1]];

D1 = [-Adelta;
      0];

G_m = ss(A1,B1,C1,D1);

%Actuator
% X = (deltaq deltaq_point) et Y = (x3 x4)
% x_3 = δ_q = u_m

A2 = [  [0, 1]
        [-(wa*wa), -2*ZetaAlpha*wa]];

B2 = [  0;
        -(wa*wa)];

C2 = [  [1 0]
        [0 1]  ];

C2bis = [ 1 0 ];

D2 = [0 ; 0];

D2bis = 0;

G_a = ss(A2,B2,C2,D2);
G_a_bis = ss(A2,B2,C2bis,D2bis);

% G_am : 
G_am = G_a_bis*G_m;

%linearisation
mdl = 'Airframe';
G_am_lin = linearize(mdl);
%iopzmap(G_am); %Affichage des zeros


%% Question 2.1: Damping gain design (5%)

%approffondir un peu cette section

% sisotool(-G_am(2,1))
C_q = -0.0606;
G_cl_q_unsc = linearize("ClosedLoop_Cq");
G_cl_q_unsc.InputName="u_unsc";
zpk(G_cl_q_unsc);


%% Question 2.2: Scaling gain design (5%)

C_sc = 1;
G = dcgain(G_cl_q_unsc);

% expliquer le pour quoi du comment au niveau de la réponse dans le scope
% simulink

%% Question 2.3: Integral gain design (10%)

C_i = 0.2; %arbitraire

% Revenir dessus parce que pas assez détaillé
% Notement les marges de phases

%% 3.	Mixed sensitivity design (60%)

%% A.	Weighting filters (5%)

%% Question 3A.1: Weighting filter discussion (2.5%)

%Pas de code discuter de W1

%% Question 3A.2: Weighting filter computation (2.5%)

%W1 et W2
%Passe haut
DC1 = 0.1;
Freq1 = 4   ;
Mag1 = 3.01;   %dB
HF1 = 100;      %rad/s 
W1 = makeweight(DC1, [Freq1, Mag1], HF1);

%passe bas
DC2 = 100;
Freq2 = 4;
Mag2 = 15;   %dB
HF2 = 0.1;      %rad/s
W2 = makeweight(DC2, [Freq2, Mag2], HF2);

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
% close all

%Création de la fonction de transfert
% non-minimum phase zero ??????????????????

% wd = 15;
ksid = 0.7;
zm = 50;    %valeur de y1 en boucle ouverte du missile
% avec ces valeures, on a une réponse en 0.17s et un dépassement qui ne vas
% pas au dessus de 1.05

Td = tf([-(wd*wd)/zm , (wd*wd)],[1 , 2*ksid*wd , wd*wd ]);

% step(Td)

% tester fmicon plus tard

%% C.	Feedback controller design (hinfsyn) (20%)

%% Question 3C.1: Controller design (10%)








