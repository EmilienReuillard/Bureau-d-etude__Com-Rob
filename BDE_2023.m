clear all
close all


%%%%%%%%%%%%%% BDE 2023 %%%%%%%%%%%%%%

%%% définition des variables
g=9.80665; %standard gravity acceleration
a=316.0561; %speed of sound at 6096m
M=3; %Mach number
Zalpha=1236.8918; %normal force derivative
Malpha=-300.4211; %Pitch moment derivative
Mq=0; %damping derivative
Zdelta=108.1144; %control force derivative
Mdelta=-131.3944; %control moment derivative
Aalpha=1434.7783; %normal acceleration derivative
Adelta=115.0529; %control acceleration derivative
Walpha=150; %actuator natural frequency
Zeta=0.7; %actuator damping ratio
V=M*a; %airspeed

s=tf("s");
%% 
%%% définition du modèle du missile
A_m=[-Zalpha/V 1 ; Malpha Mq];
B_m=[-Zdelta/V ; Mdelta];
C_m=[-Aalpha/g 0 ; 0 1];
D_m=[-Adelta/g; 0 ];
G_m=ss(A_m,B_m,C_m,D_m);
G_m.InputName="u_m";
G_m.StateName=["x1","x2"];
G_m.OutputName=["y1","y2"];
save G_m;


%%% définition du modèle de l'actionneur
A_a=[0 1 ; -Walpha^2 -2*Zalpha*Walpha];
B_a=[0; Walpha^2 ];
C_a=eye(2);
D_a=zeros(2,1);
G_a=ss(A_a,B_a,C_a,D_a);
G_a.InputName="u_cmd";
G_a.StateName=["x3","x4"];
G_a.OutputName=["u_m","udot_m"];
save G_a;

%%% linéarisation du modèle symulink Airframe

G_am=linearize("Airframe");
%[num,denom]=ss2tf(G_am.A,G_am.B,G_am.C,G_am.D);
%G_am_nz=tf(num(1,:),denom(1,:));
%G_am_q=tf(num(2,:),denom(1,:));
G_am_zpk=zpk(G_am);
%% 
%%% q 2.1
%sisotool(-G_am(2,1),1)
C_q=-0.0606;
G_cl_q_unsc=linearize("ClosedLoop_Cq");
G_cl_q_unsc.InputName="u_unsc";
zpk(G_cl_q_unsc)
zpk(G_am(1,1))

%% 
%%% Q2.2

C_sc_inv=dcgain(G_cl_q_unsc);
C_sc=1/C_sc_inv;
G=linearize("ClosedLoop_CqCsc");
zpk(G);
step(G);

%% 
%%% Q2.3

C_i=1;
G_ol_nz=linearize("ClosedLoop_CqCscCi");
zpk(G_ol_nz)
C_i_sc=C_i*C_sc*(1/s);
sisotool(G_ol_nz,1,C_q,C_i_sc)

