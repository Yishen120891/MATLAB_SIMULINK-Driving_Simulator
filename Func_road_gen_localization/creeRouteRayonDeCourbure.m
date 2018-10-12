function route = creeRouteRayonDeCourbure( rayon, distp, x0,y0,theta0, largeur)
% cr�e une route � partir de donn�es de rayon de courbure
% =>  on d�finit la route comme un ensemble de n points (nbre de valeur dans
%     vecteur rayon, espac� de la distance distp)
% route = creeRouteRayonDeCourbure( rayon, distp, x0,y0,angle0, largeur)
%
% route : route cr��e
%
% rayon : vecteur de rayons de courbure (>0 si tourne � droite) 
% distp : distance (en m�tre) entre les points du vecteur rayon de courbure
% x0,y0,theta0 : position d'origine de la route
% largeur : largeur de la voie
%
%
% une route est compos�e des champs :
%  - voieGx, voieGy : donne les positions (x,y) de la voie gauche 
%  - voieDx, voieDy : donne les positions (x,y) de la voie droite
%  - centrex,centrey : donne les positions (x,y) du centre de la voie
%  - alpha : angle de la tangente � la route (angle de cap dans r�f�rentiel x,y)      
%  - rayon : indique le rayon de courbure
%            : >0 tourne � droite
%            : +/-Inf tout droit
%            : <0 tourne � gauche
% - distp : distance (en m�tre) entre les points du vecteur rayon de courbure
%

% rayon et distance entre les points
route.distp = distp;
route.rayon = rayon;

% calcul (x,y) du milieu de voie
Dalpha = distp./rayon;
route.alpha = theta0 - cumsum(Dalpha);
route.centrex = x0 + cumsum(distp*cos(route.alpha));
route.centrey = y0 + cumsum(distp*sin(route.alpha));

% calcul voies Gauche et Droite
route.voieGx = route.centrex - largeur/2*sin(route.alpha);
route.voieGy = route.centrey + largeur/2*cos(route.alpha);
route.voieDx = route.centrex + largeur/2*sin(route.alpha);
route.voieDy = route.centrey - largeur/2*cos(route.alpha);



