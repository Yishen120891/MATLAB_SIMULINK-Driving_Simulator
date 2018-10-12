function route = creeRouteRayonDeCourbure( rayon, distp, x0,y0,theta0, largeur)
% crée une route à partir de données de rayon de courbure
% =>  on définit la route comme un ensemble de n points (nbre de valeur dans
%     vecteur rayon, espacé de la distance distp)
% route = creeRouteRayonDeCourbure( rayon, distp, x0,y0,angle0, largeur)
%
% route : route créée
%
% rayon : vecteur de rayons de courbure (>0 si tourne à droite) 
% distp : distance (en mètre) entre les points du vecteur rayon de courbure
% x0,y0,theta0 : position d'origine de la route
% largeur : largeur de la voie
%
%
% une route est composée des champs :
%  - voieGx, voieGy : donne les positions (x,y) de la voie gauche 
%  - voieDx, voieDy : donne les positions (x,y) de la voie droite
%  - centrex,centrey : donne les positions (x,y) du centre de la voie
%  - alpha : angle de la tangente à la route (angle de cap dans référentiel x,y)      
%  - rayon : indique le rayon de courbure
%            : >0 tourne à droite
%            : +/-Inf tout droit
%            : <0 tourne à gauche
% - distp : distance (en mètre) entre les points du vecteur rayon de courbure
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



