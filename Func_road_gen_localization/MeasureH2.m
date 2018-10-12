function measures=MeasureH2(xv,yv,phi_v)

global route ind_route T_horizon v stp ls Set_Animation XBord_PT YBord_PT indice_simu Uref Xv Yv
%====================================
%Trouver l'indice courant de véhicule
%------------------------------------
i=1;
norm1=calc_distance(ind_route,xv,yv);
norm2=calc_distance(ind_route+1,xv,yv);

while(norm1>norm2),
    norm1=norm2;
    i=i+1;
    norm2=calc_distance(ind_route+i,xv,yv);  
end
%disp(sprintf('i = %d\n',i));
%On change la variable globale
ind_act=ind_route+i-1;
ind_route=ind_act;
norm2=calc_distance(ind_act,xv,yv);


if ind_route > ind_act
    error('retour en arrière');
    pause;
end

% rend la courbure de la route et l'angle de cap
roref=route.courbure(ind_act);
ro_previewed=route.courbure(ind_act+round(T_horizon*v/stp));
phi_r = route.alpha(ind_act);
PsiL=phi_v-phi_r;

% ======================================================
% calculer l'indice visée,l'ecart latéral actuel et visé
% ------------------------------------------------------
P_act = [xv;yv];

%calculer l'ecart latéral actuelle: yact
voie.xg_act = route.voieGx(ind_act);
voie.yg_act = route.voieGy(ind_act);
voie.xd_act = route.voieDx(ind_act);
voie.yd_act = route.voieDy(ind_act);


P1_act = [voie.xg_act;voie.yg_act];
P2_act = [voie.xd_act;voie.yd_act];
D1_act = norm(P1_act - P_act);
D2_act = norm(P2_act - P_act);

Vect1_G = [route.centrex(ind_act) - route.voieGx(ind_act);route.centrey(ind_act) - route.voieGy(ind_act)];
Vect1_G = Vect1_G/norm(Vect1_G);
Vect1_D = [route.centrex(ind_act) - route.voieDx(ind_act);route.centrey(ind_act) - route.voieDy(ind_act)];
Vect1_D = Vect1_D/norm(Vect1_D);
Vect2_G = [P_act(1) - route.voieGx(ind_act);P_act(2) - route.voieGy(ind_act)];
Vect2_G = Vect2_G/norm(Vect2_G);
Vect2_D = [P_act(1) - route.voieDx(ind_act);P_act(2) - route.voieDy(ind_act)];
Vect2_D = Vect2_D/norm(Vect2_D);

if (D1_act<=D2_act)
    if dot(Vect1_G,Vect2_G)>=0
        yact = (route.largeur/2)-D1_act;  % norm_act;
    else
        yact = (route.largeur/2)+D1_act;
    end
else
    if dot(Vect1_D,Vect2_D)>=0
        yact = -((route.largeur/2)-D2_act); % -norm_act;
    else
        yact = -((route.largeur/2)+D2_act);
    end
end
%

yL=yact+ls*PsiL;

ind_h=ind_act+round(50/route.step); % max 50m en avant
if(ind_h>length(route.courbure))
    ind_h=length(route.courbure);
end
if(route.courbure(ind_h)>0)% à gauche
          XBord_PT = route.voieGx;
          YBord_PT = route.voieGy;  
        else if(route.courbure(ind_h)<0)% à droite
                XBord_PT = route.voieDx;
                YBord_PT = route.voieDy;
            end
end
%==========================================================================
theta_near=PsiL+yL/ls;
% changement de repère pour calculer l'angle 

x1_cal =  xv*cos(phi_r) + yv*sin(phi_r);
y1_cal = -xv*sin(phi_r) + yv*cos(phi_r);

x2_cal(ind_act:ind_h) =  XBord_PT(ind_act:ind_h)*cos(phi_r) + YBord_PT(ind_act:ind_h)*sin(phi_r);
y2_cal(ind_act:ind_h) = -XBord_PT(ind_act:ind_h)*sin(phi_r) + YBord_PT(ind_act:ind_h)*cos(phi_r);

alpha(ind_act:ind_h)= atan(((y2_cal(ind_act:ind_h)-y1_cal)./(x2_cal(ind_act:ind_h)-x1_cal)));
[alpha_min, ind_min] = min(abs(alpha(ind_act:end)));
ind_pt=ind_min(1)+ind_act-1;
Angle_PT= alpha(ind_pt); % positive à gauche

if(abs(Angle_PT*180/pi) <2.5) % on est plutot sur une ligne droite
    Angle_PT=0;
end
i_ref=1;
norm1_ref = sqrt((Xv(indice_simu)-xv)^2 + (Yv(indice_simu)-yv)^2);
norm2_ref = sqrt((Xv(indice_simu+1)-xv)^2 + (Yv(indice_simu+1)-yv)^2);

if((indice_simu+i_ref)<size(Uref,1))
    while(norm1_ref>norm2_ref)
      norm1_ref=norm2_ref;
      i_ref=i_ref+1;
      norm2_ref = sqrt((Xv(indice_simu+i_ref)-xv)^2 + (Yv(indice_simu+i_ref)-yv)^2);
    end
end
indice_simu = indice_simu + i_ref - 1;
Uref_act = Uref(indice_simu);

% figure(1)
% hold on
% plot(xv,yv,'k+')

measures=[ro_previewed ; roref ; PsiL ; ind_act ; yact ; yL ; Angle_PT ; theta_near ; Uref_act];
