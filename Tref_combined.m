clear variables
close all

addpath('Func_road_gen_localization/');
addpath('Design_controllers/');

load('external_data_Yishen/Transition_26032019.mat');

raw_data = MATDataAdapter('external_data_Yishen/Exp2_NoFog-No_Assistance.mat', 1);
% Select data with speed > v_x_min and half of whole data
v_x_min = 17;
data = raw_data.select_indices((raw_data.speed_x >= v_x_min) & (raw_data.time <= max(raw_data.time)/2));

%% --- INITIALIZATION ROAD, VEHICLE AND DRIVER MODELS
global stp EX0 EY0 EPHI0 route ind_route Smax
% Choix de la route : 1 (piste_LIVIC.mat), 2(test-script), 3(Scaner_piste.mat), 4(straight line)
pst = 5;
ind_route=1;                    % Initialization of the road index
[EPHI0,route]= Loadpiste(pst);
stp=route.step;
EX0=route.EX0;
EY0=route.EY0;

figure(1)
grid on; hold on;
plot(route.voieGx,route.voieGy,'g.');  % centre de la voie
plot(route.voieDx,route.voieDy,'r.');    % bord de la voie de droite
plot(route.centrex,route.centrey,'b.');    % bord de la voie de gauche
xlabel('X position [m]');ylabel('Y position [m]');

Smax=length(route.courbure)*route.step-510;% Longueur de la grande route

global Kp Bs v lf lr M J lw ls nt Rs mu cr0 cf0 B1 D1 A C B2 D2 Is Abic Bbic Cbic Dbic Ad Bd At B1t B2t Ct Dt C2t D22t
global L1 F H G



% % paramètres du véhicule
%         % SECOND COEFFICIENTS SET (GUI)
% %         mu = 0.8;       % Coefficient d'adhérence pour modèle linéaire
% %         v = 18;         % Vitesse longitudinal Proposer dans [5,25]m/s dans [Rah04]
% %         lf=1.127;       % Distance du centre de gravité à l'essieu avant (dans [Rah04] 1.05m)
% %         lr=1.485;       % Distance du centre de gravité à l'essieur arrière (dans [Rah04] 1.56m)
% %         lw=0.4;         % Distance en avant de l'essieu avant
% %         M=1476;         % Masse globale du véhicule
% %         J=1810;         % Moment d'inertie de lacet du véhicule, Formule initiale M*lr*lf normalement
% %         nt=0.185;       % Largeur de contact du pneu
% %         cf0=65000;      % Coefficient de raideur des pneus avant : 40000 N/rad (cf0 dans [Rah04])
% %         cr0=57000;      % Coefficient de raideur des pneus arrières : 35000 N/rad (cr0 dans [Rah04])
% %         Bs=5.7;         % Coefficient d'amortissement de la colonne de direction
% %         Is=0.05;        % Moment d'inertie de la colonne de direction
% %         Rs=16;          % Rapport de réduction de la direction
% %         Kp=0.13;        % Gain de direction manuel
% %         ls=5;           % Distance de visée du conducteur
%         mu = ExecutiveCarCS.mu_friction;       % Coefficient d'adhérence pour modèle linéaire
%         v = 18;         % Vitesse longitudinal Proposer dans [5,25]m/s dans [Rah04]
%         lf=ExecutiveCarCS.l_f;       % Distance du centre de gravité à l'essieu avant (dans [Rah04] 1.05m)
%         lr=ExecutiveCarCS.l_r;       % Distance du centre de gravité à l'essieur arrière (dans [Rah04] 1.56m)
%         lw=0.4;         % Distance en avant de l'essieu avant
%         M=ExecutiveCarCS.m;         % Masse globale du véhicule
%         J=ExecutiveCarCS.J;         % Moment d'inertie de lacet du véhicule, Formule initiale M*lr*lf normalement
%         nt=ExecutiveCarCS.eta_t;       % Largeur de contact du pneu
%         cf0=ExecutiveCarCS.C_f0;      % Coefficient de raideur des pneus avant : 40000 N/rad (cf0 dans [Rah04])
%         cr0=ExecutiveCarCS.C_r0;      % Coefficient de raideur des pneus arrières : 35000 N/rad (cr0 dans [Rah04])
%         Bs=0.57;         % Coefficient d'amortissement de la colonne de direction
%         Is=0.05;        % Moment d'inertie de la colonne de direction
%         Rs=ExecutiveCarCS.R_s;          % Rapport de réduction de la direction
%         Kp=ExecutiveCarCS.K_m;        % Gain de direction manuel
%         ls=ExecutiveCarCS.l_s;           % Distance de visée du conducteur
% 
% % Calcul du modèle véhicule (bicyclette + position sur la voie+colonne de direction)
% [Abic,Bbic,Cbic,Dbic,A,B1,B2,C,D1,D2]=Vehicle_Combined(mu,M,v,lw,J,cf0,cr0,lf,lr,ls,Kp,nt,Rs,Is,Bs);
%     % Analyse structurelle du modèle
%     if(rank(ctrb(A,[B1,B2]))~=length(A))%rank of controllability matrix
%         warning('uncontrolable system')
%     end
%     if(rank(obsv(A,C))~=length(A))%rank of controllability matrix
%         warning('unobservable system')
%     end
%     if (~all(real(eig(A))<0))
%         %warning('unstable system')
%     end
%  
%  % Calcul de la répresentation d'état de modèle conducteur (see [SCCLM13] IEEE ITS)
%  % Vecteur paramètre conducteur : [Kp Kc TI TL tau_p Kr KT TN]
%  Pconducteur=[3.4 15 1 3 0.04 1 12 0.1];
%  [Ad,Bd,Cd,Dd,Kd,x0d]=driver_model(Pconducteur,v,0,0);
% 
% % Synthèse d'un observateur d'état réduit du conducteur (utilisé pour l'assistance CoLat 2)  
%     % observateur d'état à ordre reduit
%     A11d=Ad(1:2,1:2);
%     A12d=Ad(1:2,3:3);
%     A21d=Ad(3:3,1:2);
%     A22d=Ad(3:3,3:3);
%     B1d =Bd(1:2,1:4);
%     B2d =Bd(3:3,1:4);
%     P0=eig(A11d);
%     L1 = place(A11d',A21d',3*P0)'; %3 fois plus rapide que la dynamique de conducteur
%     F=A11d-L1*A21d;
%     eg=eig(F);
%     H=B1d-L1*B2d;
%     G=F*L1+A12d-L1*A22d;
%     
% % calculer le système total: véhicule + conducteur
% Dfar=15; % Dfar = 15m
% 
% %% --- DESIGN ALL CONTROLLERS
global A B1 B2 C D1 D2
global Aw Bw Cw Lo T_horizon
global Kplus fai_0 fai_T8 fai_2T8 fai_3T8 fai_4T8 fai_5T8 fai_6T8 fai_7T8 fai_T F2 
global Kplus_a fai_0_a fai_T8_a fai_2T8_a fai_3T8_a fai_4T8_a fai_5T8_a fai_6T8_a fai_7T8_a fai_T_a F2_a 
global sw_LQ sw_DR
global KH2_Tuned
% 
% % META-DONNEES POUR TOUS LES CONTROLEURS
%     % Définition du modèle prédicteur de la courbure de la route
%     tau=0.05;
%     Aw=[-1/tau   1/tau;
%             0   -1/tau];
%     Bw=[0;
%         1/tau];
%     Cw=[1  0];
%     E=eig(Aw);
%     if(rank(obsv(Aw,Cw))~=length(Aw))%rank of observability matrix
%              warning('unobservable system')
%     end
%     
%     % The estimator dynamics should be faster than the controller dynamics
%     Lo = acker(Aw',Cw',3*E)'; %3 fois plus rapide que la dynamique de controleur
%     if (~all(real(eig(Aw-Lo*Cw))<0))
%             warning('unstable observer')
%     end
% 
%     % Time Horizon of the preview feedforward
%     T_horizon = 2;  
% 
% % CONTROLLER 1: H2-PREVIEW AUTO-STEERING
%     B22=B2(:,2);
%         
%     % Weighting parameters   
%         Qz1=[50    0 0 0;
%                0   20 0 0;
%                0    0 1 0;
%                0    0 0 1];
%         R=D1'*Qz1'*Qz1*D1;
%         Q=C'*Qz1'*Qz1*C;
%         S=C'*Qz1'*Qz1*D1;
%     
%     % Synthèse    
%     [Kplus_a, fai_0_a, fai_T8_a, fai_2T8_a,fai_3T8_a,fai_4T8_a,fai_5T8_a,fai_6T8_a,fai_7T8_a,fai_T_a,F2_a]=H2preview(A,B1,B22,R,Q,S,T_horizon,Aw,Cw);
%     disp('        -----------------------------');
%     disp('        | H2-preview autonomous: OK |');
% 
%     
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % CONTROLLER 2: H2-PREVIEW DRIVER ASSISTANCE (NEED TO RUN WITH DRIVER MODEL)    
%     %%%% ALPHA
%     %alpha = 0.6;
%     alpha_vec = 0.1:0.1:0.9;
%     %alpha2 = 3/2; %(10%->1/9 ; 20%->1/4 ; 30%->3/7 ; 40%->2/3 ; 50%->1 ; 60%->3/2 ; 70%->7/3 ; 80%->4 ; 90%->9)
%     alpha2_vec = [1/9 1/4 3/7 2/3 1 3/2 7/3 4 9];
%     Kplus_vec = zeros(4,9);
%     
%     cf=mu*cf0;
%     cr=mu*cr0;
%     a11c=-2*(cf+cr)/(M*v);
%     a12c=-1+2*(lr*cr-lf*cf)/(M*v^2);
%     a15c=2*cf/(M*v*Rs);
%     
%     index = 1;
%     while(index <=9)
%         alpha = alpha_vec(index);
%         alpha2 = alpha2_vec(index);
%         [At,B1t,B2t,Ct,Dt,C2t,D22t]=system_driver_car(Pconducteur,mu,M,v,lw,J,cf0,cr0,lf,lr,ls,Kp,nt,Rs,Is,Bs,Dfar,alpha2);
%         e=eig(At);
%         ct=ctrb(At,B1t);
%         ra=rank(ctrb(ss(At,B1t,Ct,Dt)),1e-10);
%         % Weighting parameters 
%         c4=5;
%         cyt=0; % positive: the assistance should go opposite to the lateral deviation
%         cda=-10; % negative
%         Qz=[200    0   0   0   0   0  ; % psiL
%               0   20   0   0   cyt cyt; % yact
%               0    0   3   0   0   0  ; % a
%               0    0   0   c4  0   0  ; % Td-Ta
%               0    0   0   0   1   cda; % Td
%               0    0   0   0   0   1] ; % Ta
%         R=Dt'*Qz'*Qz*Dt;
%         Q=Ct'*Qz'*Qz*Ct;
%         S=Ct'*Qz'*Qz*Dt;
% 
%         % Synthèse
%         [Kplus, fai_0, fai_T8, fai_2T8,fai_3T8,fai_4T8,fai_5T8,fai_6T8,fai_7T8,fai_T,F2]=H2preview(At,B1t,B2t,R,Q,S,T_horizon,Aw,Cw);
%         disp('        | H2-preview assistance: OK |');
%         disp('        -----------------------------');
%         Kplus
% 
%         [ Avr , Bvr , Cvr , Dvr ] = Vehicle( mu,M,v,lw,J,cf0,cr0,lf,lr,ls,Kp,nt,Rs,Is,Bs );
% 
%     %    
%         [Mh2.A,Mh2.B,Mh2.C,Mh2.D] = linmod('Modele_optimisation');
%         Mh2 = ss(Mh2.A,Mh2.B,Mh2.C,Mh2.D);
%         Mh2.StateName = {'beta','r','psi_l','yl','deltad','deltad_pt','x_1d','x_2d','Td','beta_vr','r_vr','psi_l_vr','yl_vr','deltad_vr','deltad_pt_vr','x_sub1','x_sub2','x_sub3','x_sub4','x_sub5','x_sub6','x_sub7','x_sub8','x_sub9','x_sub10','x_sub11'};
%         Mh2.InputName = {'Fw' ,  'rho_ref','u'};
%        % Mh2.OutputName = {'z_psi_L','z_y_act','z_a','z_Ta-alphaTd','z_Td','z_Ta','y_beta','y_r','y_psi_L','y_y_L','y_delta_d','y_deltad_pt'};
%         Mh2.OutputName = {'z_psi_L','z_y_act','z_a','z_Ta-alphaTd','z_Td','z_Ta','y_r','y_psi_L','y_y_L','y_delta_d'};
% 
%         KH2 = ltiblock.gain('KH2',zeros(1,4));
%         %KH2.InputName = {'y_beta','y_r','y_psi_L','y_y_L','y_delta_d','y_deltad_pt'};
%         KH2.InputName = {'y_r','y_psi_L','y_y_L','y_delta_d'};
%         KH2.OutputName = {'u'};
% 
%         Mh2_KH2 = connect(Mh2,-KH2,{ 'Fw' ,  'rho_ref'},{'z_psi_L','z_y_act','z_a','z_Ta-alphaTd','z_Td','z_Ta','y_r','y_psi_L','y_y_L','y_delta_d'});
%         %Mh2_KH2 = connect(Mh2,-KH2,{ 'Fw' ,  'rho_ref'},{'z_psi_L','z_y_act','z_a','z_Ta-alphaTd','z_Td','z_Ta','y_beta','y_r','y_psi_L','y_y_L','y_delta_d','y_deltad_pt'});
%     %
%         ReqH2 = TuningGoal.LQG({ 'Fw' ,'rho_ref'},{'z_psi_L','z_y_act','z_a','z_Ta-alphaTd','z_Td','z_Ta'},eye(2),eye(6));
%         options = systuneOptions('RandomStart',10);
%         [Mh2_Tuned,fSoft,gHard,INFO] = systune(Mh2_KH2,ReqH2,options);
%         KH2_tuned = ss(Mh2_Tuned.Blocks.KH2);
%     %
%         [SS_Mh2_Tuned,fval] = evalSpec(ReqH2,Mh2_Tuned,INFO);
%         %Kplus = [ KH2_tuned.D(1) KH2_tuned.D(2) KH2_tuned.D(3) KH2_tuned.D(4)  KH2_tuned.D(5) KH2_tuned.D(6)]
%         Kplus_vec(:,index) = [ KH2_tuned.D(1) KH2_tuned.D(2) KH2_tuned.D(3) KH2_tuned.D(4)]
%         fSoft
%         gHard
%         index = index+1;
%     end
% 
%     index = 1:9;
%     Kplus_r = polyfit(alpha_vec,Kplus_vec(1,:),6);
%     Kplus_psiL = polyfit(alpha_vec,Kplus_vec(2,:),6);
%     Kplus_yL = polyfit(alpha_vec,Kplus_vec(3,:),7);
%     Kplus_deltad = polyfit(alpha_vec,Kplus_vec(4,:),6);
%     alpha =0.5;
%     
%    % polyval(Kplus_r,0.3)


alpha = 0.5;

    %% --- CHOIX DU MODE DE CONDUITE : H2-PREVIEW ASSIS. + DRIVER (1), H2-PREVIEW SEUL (2), DRIVER SEUL (3)    
cmd=3;
disp('-------------------------------------------------');
if(cmd==1) % H2-Preview for assistance
    sw_DR=1;
    sw_LQ=1;
    disp('| Driver and H2-preview assistance are driving! |');
end
if(cmd==2)  % H2-Preview auto-steering
    sw_DR=0;
    sw_LQ=1;
    disp('|          H2-preview only is driving           |');
end
if(cmd==3) % Driver model
    sw_DR=1;
    sw_LQ=0;
    disp('|            Driver only is driving!            |');
end
disp('-------------------------------------------------');

% 
%% --- PREPARATION DE LA SIMULATION
global Bs v lf lr M J lw ls nt Rs mu cr0 cf0 A1 B1 C1 D1 A C B2 D2 Is Abic Bbic Cbic Dbic 
global EX0 EY0 EPHI0 route ind_route Smax Tt XBord_PT YBord_PT
% global AG1 AG2 BG1 BG2 CG1 CG2 DG1 DG2
global Ad Bd At B1t B2t Ct Dt
global Aw Bw Cw Lo T_horizon ind_route
global sw_LQ sw_DR stp %sw_pid 
global Kplus KH2_Tuned fai_0 fai_T8 fai_2T8 fai_3T8 fai_4T8 fai_5T8 fai_6T8 fai_7T8 fai_T F2 
global Kplus_a fai_0_a fai_T8_a fai_2T8_a fai_3T8_a fai_4T8_a fai_5T8_a fai_6T8_a fai_7T8_a fai_T_a F2_a
global L1 F H G  
global Kplus_beta Kplus_r Kplus_psiL Kplus_yL Kplus_deltad Kplus_deltad_pt


% valeurs initiales véhicule 
vehicule.x0 = route.EX0;                    %
vehicule.y0 = route.EY0;                    % position initiale de la voiture
vehicule.theta0 = EPHI0;   %pi/2;% orientation initiale de la voiture
vehicule.K = -1/(48*pi);            % facteur angle volant (ici choisi pour rayon de braquage de R=12m pour 2 tours de volant (alpha_v=4*pi) : K=1/(R*alpha_v))
vehicule.phi_max = pi/2;            % angle maximal pour la rotation des roues (est inférieur à pi/2 sur un véhicule normalement constitué)

% Initialisation des paramètres conducteur
global tau_l tau_p Kl Wn ksi
global conducteur Uref_actu ind_route1 route1

tau_l=1;            % Attention du conducteur à vitesse moyenne: 1s
tau_s=0.5;          % Attention du conducteur a vitesse faible : 0.5s 
tau_p=0.151;        % Temps de réaction : 0.151s
Kl=20;              % Gain proportionnel
Wn=10;              % Fréquence propre : 10 rad/s
ksi=0.707;          % Coefficient d'amortissement : sqrt(2)/2
Uref_actu = 0;
indice_simu = 1;
ind_route = 1;
route1 = route;
ind_route1 = 1;

Tt = 0.02;
conducteur.Te = Tt;      % temps d'échantillonnage
conducteur.vitesse = v;  % vitesse conducteur
conducteur.theta0 = 0;   % Motivation du choix de cette valeur inconnue - Fabien Claveau 2006

XBord_PT = route.voieDx;
YBord_PT = route.voieDy;

assignin('base','ind_route',ind_route);
assignin('base','route',route);

nom = 'Partage_Simulink';
simut = run_simulation(route,vehicule,conducteur, nom);

%% --- AFFICHAGE DES RESULTATS
    % Tracé de la trajectoire du véhicule
    figure(1)
    grid on;
    hold on;
    plot(simut.Xv,simut.Yv,'m.');
%     plot(simut.Xv1,simut.Yv1,'k.');
    plot(data.vehicle_rear_axle_center_x, data.vehicle_rear_axle_center_y, 'k.');
    xlabel('X position [m]');ylabel('Y position [m]'); 
    hold off;
 
    % Affichage des caractéristiques principales en termes de suivi de voie
    [M,I]=max(abs(simut.yact(1:end)));
    disp('----------------------------------------------');
    disp('|    Simulation - Performance indicators:    |');
    disp(sprintf('|    max(erreur latérale) : %.2f m           |', max(abs(simut.yact(1:end)))));
    disp(sprintf('|    max(couple) : %.2f N.m                 |',max(abs(simut.Tv))));
    disp(sprintf('|    max(couple_conducteur) : %.2f N.m      |',max(abs(simut.Td))));
    disp(sprintf('|    max(couple_assistance) : %.2f N.m      |',max(abs(simut.Ta))));
    disp(sprintf('|    max(acceleration) : %.2f m/s²           |',max(abs(simut.err_acc))));
    disp(sprintf('|    moyen(erreur latérale) : %.2f m         |',mean(abs(simut.yact))));
    disp('----------------------------------------------');
    
    disp(sprintf('|    indicemax(erreur latérale) : %.2f m           |', I));
    % Tracé de l'erreur de déplacement et du couple volant total
    figure (2)
    plot(simut.tmp,simut.yact,'r','LineWidth',1.5);
    hold on;
    plot(simut.tmp,simut.Tv,'LineWidth',1.5);
    legend('erreur deplacement','Couple volant','Location','NorthEast')
    xlabel('Time in seconds');
    hold off;
 
    % Tracé des différents couples appliqués à la colonne de direction
    figure (3)
    plot(simut.temps,simut.Tv,'k','LineWidth',1.5);
    hold on
    plot(simut.temps,simut.Ta,'r')
    plot(simut.temps,simut.Td,'g')
    hold off
    title('Torque applied on the steering column');
    legend('Total Torque Tv','Assistance Torque','Driver Torque');
    xlabel('Time in seconds');
    ylabel('Torque in N.m');
    hold off
%     
%     % Sauvegarde  du temps, erreur latéral, couples total, conducteur, assistance
%     x_tmp_drdlyass=simut.tmp;
%     y_act_drdlyass=simut.yact;
%     y_Tv_drdlyass=simut.Tv; %couple total
%     y_Td_drdlyass=simut.Td; %couple conducteur
%     y_Tc_drdlyass=simut.Ta; %couple H2 ou assistance
% %    y_fir = simut.FIR;
%     save save_drdlyass.mat x_tmp_drdlyass y_act_drdlyass y_Tv_drdlyass y_Td_drdlyass y_Tc_drdlyass% y_fir
%     
% tf=size(x_tmp_drdlyass);
% Tcoh = zeros(tf(1),1);
% Tres = zeros(tf(1),1);
% Tcont = zeros(tf(1),1);
% figure(10)
% hold on
% for i=1:tf(1)
%     if (y_Tc_drdlyass(i)*y_Td_drdlyass(i)>=0)
%         Tcoh(i) = 1;
%     end
%     if (y_Tc_drdlyass(i)*y_Td_drdlyass(i)<0)&&(y_Tc_drdlyass(i)<=y_Td_drdlyass(i))
%         Tres(i) = 1;
%     end
%     if (y_Tc_drdlyass(i)*y_Td_drdlyass(i)<0)&&(y_Tc_drdlyass(i)>y_Td_drdlyass(i))
%         Tcont(i) = 1;
%     end
%     figure(10)
%     hold on
%     plot(i,sum(Tcoh(1:i))/i,'g.');
%     plot(i,sum(Tres(1:i))/i,'b.');
%     plot(i,sum(Tcont(1:i))/i,'r.');
% end
% 
% figure(11)
% hold on
% plot(x_tmp_drdlyass,Tcoh,'g');
% plot(x_tmp_drdlyass,Tres,'b');
% plot(x_tmp_drdlyass,Tcont,'r');
% 
% Tcoh_total = sum(Tcoh)/tf(1)
% Tres_total = sum(Tres)/tf(1)
% Tcont_total = sum(Tcont)/tf(1)
