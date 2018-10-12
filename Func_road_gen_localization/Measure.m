function measures=Measure(xv,yv,phi_v)

global route1 ind_route1 ls
%====================================
%Trouver l'indice courant de véhicule
%------------------------------------
i=1;
norm1=calc_distance(ind_route1,xv,yv);
norm2=calc_distance(ind_route1+1,xv,yv);
% 
% figure (50)
%          hold on; 
%          plot(route1.centrex,route1.centrey,'c');

while(norm1>norm2),
    norm1=norm2;
    i=i+1;
    norm2=calc_distance(ind_route1+i,xv,yv);  
end
%disp(sprintf('i = %d\n',i));
%On change la variable globale;
ind_act=ind_route1+i-1;
ind_route1=ind_act;

if ind_route1 > ind_act
    error('retour en arrière');
    pause;
end

% rend la courbure de la route et l'angle de cap
phi_r = route1.alpha(ind_act);
PsiL=phi_v-phi_r;

% ======================================================
% calculer l'indice visée,l'ecart latéral actuel et visé
% ------------------------------------------------------
P_act = [xv;yv];

%calculer l'ecart latéral actuelle: yact
voie.xg_act = route1.voieGx(ind_act);
voie.yg_act = route1.voieGy(ind_act);
voie.xd_act = route1.voieDx(ind_act);
voie.yd_act = route1.voieDy(ind_act);


P1_act = [voie.xg_act;voie.yg_act];
P2_act = [voie.xd_act;voie.yd_act];
D1_act = norm(P1_act - P_act);
D2_act = norm(P2_act - P_act);

Vect1_G = [route1.centrex(ind_act) - route1.voieGx(ind_act);route1.centrey(ind_act) - route1.voieGy(ind_act)];
Vect1_G = Vect1_G/norm(Vect1_G);
Vect1_D = [route1.centrex(ind_act) - route1.voieDx(ind_act);route1.centrey(ind_act) - route1.voieDy(ind_act)];
Vect1_D = Vect1_D/norm(Vect1_D);
Vect2_G = [P_act(1) - route1.voieGx(ind_act);P_act(2) - route1.voieGy(ind_act)];
Vect2_G = Vect2_G/norm(Vect2_G);
Vect2_D = [P_act(1) - route1.voieDx(ind_act);P_act(2) - route1.voieDy(ind_act)];
Vect2_D = Vect2_D/norm(Vect2_D);



if (D1_act<=D2_act)
    yact = -(route1.largeur/2)+D2_act;
else
    yact = -(-(route1.largeur/2)+D1_act);
end
%

yL=yact+ls*PsiL;

%==========================================================================


measures=[ PsiL ; yL ];
