%Ceci est le fichier du BDE de Commande robuste

%Q1
%Constantes
g = 9.81;
a = 316.0561;
M = 3;
V = M; %parce que connard peut pas garder la meme letttre ptn
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

A1 = [[-Zalpha/V, 1]
      [Malpha, Mq]];
  
B1 = [ -Zalpha/V; 
        Mdelta ];
  
C1 = [[-Aalpha/g, 0]
      [0, 1]];

D1 = [-Adelta;
      0];
  
sys1 = ss(A1,B1,C1,D1);

%Actuator


