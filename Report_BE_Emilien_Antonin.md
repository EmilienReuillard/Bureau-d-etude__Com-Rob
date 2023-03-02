# BE Commande Robuste Emilien Reuillard et Antonin Renoir

## Question 1.1: Flight dynamics (10%)

Nous allons discuter du modèle linéaire nominal sans incertitude de l'espace d'état de la dynamique de tangage du missile. 
En effet le modèle du missile donné dans l'enoncé est purrement linéaire car il ne comporte pas de composantes non linéaires comme des termes au carré ou alors des fonctions non linéaire comme des cosinus ou des sinus (fonctions souvent présentes dans les systèmes non linéaires lorsque des angles interviennent)
Plusieurs étapes sont néccéssaires pour obtenir le modèle linéaire à partir du modèle non-linéaire. 
La première étape est la formulation des équations de la dynamique de tangage du missile : Les équations de la dynamique de tangage du missile sont des équations différentielles non linéaires qui décrivent l'évolution de l'angle de tangage, de la vitesse angulaire et d'autres variables liées au mouvement du missile. Ces équations peuvent être obtenues à partir des lois de la physique et de la dynamique du missile, en utilisant des modèles mathématiques appropriés. 
Nous les avons vu dans le cours et il n'est pas intérréssant de les rappeler dans ce rapport. 
La deuxième étape est la linéarisation de ces équations. Pour cela il est judicieux de savoir quels thermes linéarisé ainsi que le point d'équilibre autour duquel il faut linéarisé. En effet le linéarisation d'une fonction non-linéaire se fait toujours au voisinage d'un point d'équilibre. La linéarisation le plus connu dans ce cas concerne les fonctions trigonométrique cosinus(alpha) et sinus(alpha) qui se linéarise respectivement en 1 et alpha lorsque alpha est proche (voisinage) de zéro (point d'équilibre).
Pour le cas du missile cela est un peu plus compiqué. 

Calcul du (ou des) point(s) d'équilibre : en général dans un système non linéaire les points d'équilibres se calculs de la mnière suivante. 
Soit un système non linéaire représenté sous forme de représentation d'état non linéaire (les matrices sont des fonction avec en paramètre le vecteur d'état) alors un point d'équibre (valeur particulière du vecteur d'état) est un point pour lequel la dérivé du vecteur d'état est nul. 
Soit par example ([x1, x2] vecteur d'état) x1_dot = f_1(x1,x2) = 0 et x2_dot = f_2(x1,x2) = 0

Dans un premier temps il faut traiter de **l'enveloppe de vol**. Effet d'apres le texte de Reichert, 1992 [1], il faut que l'angle d'attaque soit compris entre -20° et 20° de manière a assurer la stabilité robuste du système bouclé et assurer la poursuite d'un signal d'entré de type échelon. Cela nous permet donc de savoir ou linéariser les équations fournis dans le cours. Alpha autour de ces deux extremums. Ensuite pour linéariser nous utilisons la décomposition en série de Taylor qui permet de décomposoer les composantes non linéaires suivnt plusieurs degrées. Ainsi le premier terme peut être gardé pour apliquer la linéarisation. 

Sous Matlab /Simulmink : 
- La fonction linmod() : cette fonction permet de générer automatiquement une représentation linéaire d'un modèle Simulink.
- La commande linearize() : cette commande permet de linéariser un modèle Simulink ou une fonction Matlab en utilisant la méthode de la réponse fréquentielle linéaire. Nous utiliserons cette fonction par la suite. 
- La commande ss peut aussi servir à linéariser en créant le modèle d'état. 


## Question 1.2: Model construction & analysis (10%)
Pour obtenir G_am nous avons changer la matrice C dans la representation d'état de l'Actuator pour ne visualiser que x3. Ainsi la representation d'état de G_am est : 

![G_am](./Ressources/G_am.png)

Nous avons créé le schéma sous simulink : Voir **Airframe.slx**

Les formes zpk du système de u_cmd à y1, y2 respectivement sont (par exemple G_am_nz, G_am_q) sont : 
![Les formes zpk du système de u_cmd à y1, y2 respectivement](./Ressources/formes_zpk_1.2.png)

Voici les pôles et les zéros d'entrée / sortie avec la fonction isomap : 
![isomap(G_am_lin)](./Ressources/isomap(G_am_lin).png)
Pour la sortie y1 nous avons 4 pôles et 2 zéros. Alors que pour y2 comme sortie nous avons 4 pôles et 1 zéro. 
Les composantes instables sont celles avec des parties réels positives. 


## 2. Loopshaping(20%)
## Question 2.1: Damping gain design (5%)

En se basant sur le ficgure numéro 3 on cherche le gain C_q tel que l'amortissement soit optimisé (0.7). Poour cela on utilise rlocus qui trace la position des pôle du système en faisant varier le gain c_q de 0 à l'infini. 
On obtient les figures ci-après. 
![rlocus(G_ol_q)](./Ressources/rlocus(G_ol_q).png)
En zommant on voit que on peut placer le curser pour voir quel gain correspond à l'ammortissemnt de 0.7 : 
![rlocus(G_ol_q)_zoom](./Ressources/rlocus(G_ol_q)_zoom.png)
(Cependant avec le precision du graphique nous n'avojs pas réussi à nous placer au proche de 0.7). Donc nous avons estimé le gain souhaité égale à -0.16.

Avec rlocus la courbe du haut nous fourni aussi une valeur de C_q pour un amortissement de 0.7 mais ce gain est compris en 0 et 0.041. Ainsi il nous a parru plus judicieux de choisir les courbes prochent de l'axe des abscisses pour rlocus.  

Ensuite nous pouvons comparer les resultat obtenu avec la figure 3 reproduite : 
- zpk(G_am(1,1)):
![zpk(G_am(1,1))](./Ressources/zpk(G_am(1,1).png)
- zpk(G_cl_q_unsc) :
![zpk(G_cl_q_unsc)](./Ressources/zpk(G_cl_q_unsc).png)

Les zeros sont triviaux à trouver et les pôles peuvent être donnés avec la fonction *pole*. 

![pole(G_am(1,1))](./Ressources/pole(G_am(1,1)).png)
![pole(G_cl_q_unsc).png](./Ressources/pole(G_cl_q_unsc).png)


## Question 2.2: Scaling gain design (5%)
On récurpère l'inverse du gain C_sc avec la foncion dcgain sur G_cl_q_unsc. On le nomme C_sc_inv que l'on inverse pour avoir C_sc. 

On implemente le gain C_q et C_sc dans Simulink. Et on récupère *G* le système linéarisé qui prend en compte les deux gains précedants. On va alors vérifier la convergence de G : 


Pour verifier la 
## Sources 
[1] : R. T. Reichert, “Dynamic scheduling of modern-robust-control autopilot designs for missi- les”, IEEE Control Systems, vol. 12, no. 5, pp. 35–42, 1992.
