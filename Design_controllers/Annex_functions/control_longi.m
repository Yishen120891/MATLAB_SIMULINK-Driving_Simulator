function sortie = control_longi(rho_PT,d,T)

global v A B C D mu ls GAMAb VLIM VIT Dis 

gamal = 4;
tr = 0;
vmax_cond = 10;

if rho_PT == 0
    rho_PT = 0.01;
else
    rho_PT = rho_PT;
end


vlim = sqrt(gamal/abs(rho_PT));

if vlim > 40
    vlim = 40;
else
    vlim = vlim;
end

if d == 0
    d = 5;
else
    d = d;
end

VLIM =[VLIM vlim];

if vlim < v  
    gama_b = (vlim^2-v^2)/(2*(d-v*tr));
    vk = gama_b*T + v;
    
elseif v < vmax_cond
    
    gama_b = (vmax_cond^2-v^2)/(2*(d-v*tr));
    vk = gama_b*T + v;
else
    vk = v;
    gama_b = 0;
end

GAMAb = [GAMAb gama_b];

v = vk;

VIT = [VIT v];
Dis = [Dis d];
% Longueurs caractéristiques du véhicule
lf=1.00;                  % Distance du centre de gravité à l'essieu avant (dans [Rah04] 1.05m)
lr=1.50;                  % Distance du centre de gravité à l'essieur arrière (dans [Rah04] 1.56m)
lw=0.4;                   % Distance en avant de l'essieu avant

% Masses caractéristiques du véhicule
M=1500;                   % Masse globale du véhicule
m=M;
m_f=(M*lr)/(lf+lr);       % Valeur nominale de la masse répartie sur les roues avants 900 kg
m_r=(M*lf)/(lf+lr);       % Valeur nominale de la masse répartie sur les roues arrières 600 kg   

% Moment d'inertie de lacet du véhicule 
J=(m_f+m_r)*lr*lf;        % Pour une masse de 1500kg lf=1 et lr=1.5 J vaut 2250

% Caractéristiques des roues du véhicules
R = 0.3;                  % Rayon des roues du véhicule en m
Ip = 1.25;                % Moment d'inertie des roues du véhicules en kg.m²
Cacc = 0;                 % Couple accélérateur au niveau des roues avants
nt=0.13;                  % Largeur de contact du pneu

% Coefficients de raideurs des pneus
% cf0=40000*mu;             % Coefficient de raideur des pneus avant : 40000 N/rad (cf0 dans [Rah04])
% Cf=cf0;
% cr0=35000*mu;             % Coefficient de raideur des pneus arrières : 35000 N/rad (cr0 dans [Rah04])
% Cr=cr0;

Cyav = 47135;
Cyar = 56636;
cf0 = Cyav*mu;
cr0 = Cyar*mu;
Cf=cf0;
Cr=cr0;

Bs=10;              % Coefficient d'amortissement de la colonne de direction
Js=1;               % Attention, coeexistence Is et Js pour le moment d'inertie
Is=Js;
Rs=21;              % Rapport de réduction de la direction;
Kp=1;               % Gain de direction manuel


% Calcul des paramètres du modèle linéaire
% Le modèle est donnée dans la thèse [Rah04 - chap. 1, pp. 124/135]
    % Calcul des coefficients des matrices A et B
    e12=1/(M*v);
    e22=lw/J;
    a11=-2*(cf0+cr0)/(M*v);
    a12=-1+2*(lr*cr0-lf*cf0)/(M*v^2);
    a21=2*(lr*cr0-lf*cf0)/J;
    a22=-2*(lr^2*cr0+lf^2*cf0)/(J*v);

    b1=2*cf0/(M*v);
    b2=2*(cf0*lf)/J;

    % Définition de la matrice A
    % Etat=[beta; r; PsiL; Yl]
    
    A=[a11 a12  0  0;
       a21 a22  0  0;
         0   1  0  0;
         v  ls  v  0];

    % Définition de la matrice B
    % La commande possède 3 entrées :
    %u=[fw;
    %   roref; 
    %   deltaf]; 
    Bu=[b1;
        b2;
        0;
        0];
%% fw
    Bw1=[e12;
         e22;
         0;
         0];
    %% rho_ref 
    Bw2=[0;
         0;
        -v;
         0];
 
    B=[Bu,Bw1,Bw2];
    
    % Définition de la matrice C
        % Calcul de l'aligning torque
        Ts_beta=2*Kp*cf0*nt/Rs;
        Ts_r=2*Kp*cf0*lf*nt/(Rs*v);
        Ts_deltaf=2*Kp*cf0*nt/Rs;
        K_Ts=[-Ts_beta -Ts_r Ts_deltaf];

        % Définition de la matrice C
        C=[ 1   0    0    0;
            0   1    0    0;
            0   0    1    0;
            0   0    0    1;
     -Ts_beta  -Ts_r    0    0];

    % Définition de la matrice D
    D =        [0   0   0;
                0   0   0;
                0   0   0;
                0   0   0;
   2*Kp*cf0*nt/Rs   0   0 ];

sortie = [vlim v];

