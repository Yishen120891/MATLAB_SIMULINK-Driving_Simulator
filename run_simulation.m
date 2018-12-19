function simu = run_simulation( route, vehicule, conducteur, nom)
% lance la simulation (et récupère les bons paramètres)
%
% simu = run_simu( route, vehicule, conducteur, nom)
%
% route : route
% vehicule : vehicule
% conducteur : celui qui conduit (nan, je déconne)
% nom : nom de la simulation (optionnel)

persistent numerosimu;

% cherche le modèle simulink
simuName=nom; %simu 
open_system(nom);

if isempty( find_system(simuName) )
    open_system( simuName);
end

if nargin==3
    if isempty(numerosimu)
        numerosimu=1;
    end    
    nom = [ 'Driver_model_pt' num2str(numerosimu) ];        
    numerosimu = numerosimu + 1; 
end

% fixe les paramètres de simulation
simu.route=route;
simu.conducteur=conducteur;
simu.vehicule=vehicule;
simu.nom = nom;

% calcule les conditions initiales
% loca = perception_PT(route,conducteur,vehicule.x0,vehicule.y0,vehicule.theta0,1,0);
% simu.conducteur.theta0=loca(1);
% simu.conducteur.dist0=loca(2);

assignin('caller','simuparam',simu);
%lance la simulation
[simu.temps] = sim(simuName);
% récupère les données de la simu et les stocke dans simu
simu.roref = ro_ref;
simu.fw = fw;
simu.r = r;
simu.PsiL = PsiL;
simu.Ts = Ts;
simu.Tv = Tv;
simu.Td = Td;
simu.Ta = Ta;
simu.deltad = deltad;
simu.deltadp = deltadp;
simu.Xv = Xv;
simu.Yv = Yv;
simu.Xv1 = Xv1;
simu.Yv1 = Yv1;
simu.yL = yL;
% simu.indice_route_courant = indice_route_courant;
simu.yact = yact;
simu.err_acc = err_acc;
%simu.wanted_acc = wanted_acc;
simu.tmp = tmp;
simu.theta_far = theta_far;
simu.theta_near = theta_near;
simu.beta = beta1;
simu.delta_SW = delta_SW;

simu.RawData = RawData;
simu.DriverModelRawData = DriverModelRawData;