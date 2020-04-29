function [EPHI0,route]=Loadpiste(piste)

switch piste
case 1
    load piste_LIVIC.mat; % Piste LIVIC
    %----------------------------------
    % piste = 
    % 
    %          x: [6780x1 double]: x du centre de la route = x du bord de (voie gauche) à droite (parcours dans sens trigonométrique)
    %          y: [6780x1 double]: y du centre de la route = y du bord de (voie gauche) à droite (parcours dans sens trigonométrique)
    %         xg: [6780x1 double]: x du bord de la route à gauche(parcours dans sens trigonométrique)
    %         yg: [6780x1 double]: x du bord de la route à gauche(parcours dans sens trigonométrique)
    %        xcg: [6780x1 double]: x du centre de la voie à gauche
    %        ycg: [6780x1 double]: y du centre de la voie à gauche
    %         xd: [6780x1 double]
    %         yd: [6780x1 double]
    %        xcd: [6780x1 double]
    %        ycd: [6780x1 double]
    %        cap: [6780x1 double]: angle cap de la piste 
    %       topo: [6780x1 double]
    %     topocd: [6780x1 double]
    %     topocg: [6780x1 double]
    %        Cap: [6780x1 double]
    %      courb: [6780x1 double]: courbure de la piste
    %----------------------------------
    %% changer la longueur de la piste  
        step = 0.5;
        ro_ref0 = piste.courb'; 
        
%      figure(10)
%      hold on
%     route.courbure = ro_ref0;
%      route.courbure = zeros(1,size(ro_ref0,2)*2);
%      route.courbure (1,1) = ro_ref0(1,1);
%      for i=1:(size(ro_ref0,2)-1)
%          route.courbure (1,2*i+1) = (ro_ref0(1,i)+ro_ref0(1,i+1))/2;
%          route.courbure (1,2*i) = ro_ref0(1,i);
%      end
%      route.courbure (1,2*size(ro_ref0,2))=ro_ref0(1,size(ro_ref0,2));
%      ro_ref0 = route.courbure;
% % %     


%step=step/128;



%     for i=1175:size(route.courbure,2)
%         if route.courbure(i) > 0
%             route.courbure(i) = 1/((1/route.courbure(i)) - 1.75);
%         elseif route.courbure(i) < 0
%             route.courbure(i) = -1/((1/abs(route.courbure(i))) + 1.75);
%         end
%     end
%     plot(route.courbure,'r')
        route.courbure = ro_ref0;
        x1 = 1:0.5:3339;
        x2 = 1:0.05:3339;
        y1=courbu(route,x1,step);
        ro_ref0 = interp1(x1,y1,x2,'spline');
        step=step/10;
        route.rayon = -1./ro_ref0;
        route = creeRouteRayonDeCourbure( route.rayon,step, 0,0,-0.230255481541, 3.5);
        route.courbure = ro_ref0;
        route.courbure = route.courbure'; 
        
        route.step = step;
        EPHI0=-0.230255481541; 
        route.EX0=0;
        route.EY0=0;
        route.largeur = 3.5;
    
 case 2 %test
    %profil de courbure
    % minimum vitesse 60Km/h= 16.7 m/s
    % max Acceleration laterale= 4 m/s2 ==>     % Rmin=V^2_min/Accel_max=16.7^2/4=69.7m
    circle=-1/70;%circle, virage à gauche
    partie_sin=1/100;
    step=0.1;
    distance_chngement_R=100;
    route.largeur = 3.5;

    % virage 1= circle
    u=zeros(225/step,1);%225
%     u=zeros(100/step,1);
    % virage 2= Partie sinus
    distance_changemnt_tan=150;
    distance_virage=100;
    for i=1:1
        u=[u;linspace(0,-partie_sin,distance_chngement_R/step)'];
        u=[u;linspace(-partie_sin,partie_sin,distance_changemnt_tan*0.5/step)'];
        u=[u;partie_sin*ones(distance_virage/step,1)];
        u=[u;linspace(partie_sin,-partie_sin,distance_changemnt_tan/step)'];
        u=[u;linspace(-partie_sin,0,distance_chngement_R/step)'];
    end
    
    for i=1:1
        u=[u;linspace(0,-partie_sin,distance_chngement_R/step)'];
        u=[u;linspace(-partie_sin,partie_sin,distance_changemnt_tan/step)'];
        u=[u;partie_sin*ones(distance_virage/step,1)];
        u=[u;linspace(partie_sin,-partie_sin,distance_changemnt_tan*0.5/step)'];
        u=[u;linspace(-partie_sin,0,distance_chngement_R/step)'];
    end
        u=[u;zeros(20/step,1)];
        u=[u;zeros(225/step,1)];%225
        %-----------
        [b,a] = butter(2,0.002);
        u=1.4*filter(b,a,u);
        R = 1./u;
        route=creeRouteRayonDeCourbure( R,step, 0,0,0, 3.3);
        route.centrex = route.centrex(1:1:end);
        route.centrey = route.centrey(1:1:end);
        route.voieDx = route.voieDx(1:1:end);
        route.voieDy = route.voieDy(1:1:end);
        route.voieGx = route.voieGx(1:1:end);
        route.voieGy = route.voieGy(1:1:end);
        route.alpha = route.alpha(1:1:end);
        route.rayon = route.rayon(1:1:end);
        route.courbure = -1./route.rayon;
        route.courbure=route.courbure'; 
        route.step=0.1;
        route.largeur = 3.5;
        % detection Type de geometrie
        route.Type_geo = type_geometrie(route.courbure);
        EPHI0=0;
        route.EX0=0;
        route.EY0=0;
       
    case 3
        % problème de courbure dans la sauvegarde : ne coincide pas avec x
        % y, et est fortement bruitée
        % regénération de la piste sur la base de la courbure seule (filtrée)
        step = 0.69;
        load ('Scaner_piste.mat'); %simulateur piste
        [b,a] = butter(2,0.15);    % filtrage de la courbure, trop bruitée
        ro_ref0=filter(b,a,ro_ref0); 
        route.rayon = -1./ro_ref0;
        route = creeRouteRayonDeCourbure( route.rayon,step, 0,0,0, 3.5);
        route.courbure = ro_ref0; %pour etre d'accord avec la convention "positive à gauche"
        route.courbure = route.courbure'; 
        route.step = step;
        EPHI0=0; 
        route.EX0=0;
        route.EY0=0;
        route.largeur = 3.5;
        %         load ('Scaner_piste.mat'); % Piste disponible sous Scaner Studio (simulateur)
        %         distp=0.75;
        %         route.EX0=1.7288-0.088;
        %         route.EY0=-3.3129+0.083;                  %-1.75;
        %         route.step=distp;
        %         route.distp=distp;
        %         route.alpha = RoadAngle;
        %         route.centrex = center_x;
        %         route.centrey = center_y;
        %         route.centrex(1) = center_x(2)-0.088;
        %         route.centrey(1) = center_y(2)+0.083;
        %         route.voieGx = left_bordX;
        %         route.voieGy = left_bordY;
        %         route.voieDx = right_bordX;
        %         route.voieDy = right_bordY;
        %         route.voieDx(1) = route.voieDx(2);
        %         route.voieDy(1) = route.voieDy(2)+0.0003;
        %         route.courbure = ro_ref0; %pour etre d'accord avec la convention "positive à gauche"
        %         route.courbure=route.courbure';
        %         % detection Type de geometrie
        %         %route.Type_geo = type_geometrie(route.courbure);
        %         %x=route.Type_geo;
        %         EPHI0=pi/2;
    
    case 4
        step = 0.1;
        
        u = zeros(floor(1000/step),1);
        R = 1./u;
        largeur = 3.5;
        route = creeRouteRayonDeCourbure( R,step, 0,0,0, largeur);
        route.centrex = route.centrex(1:1:end);
        route.centrey = route.centrey(1:1:end);
        route.voieDx = route.voieDx(1:1:end);
        route.voieDy = route.voieDy(1:1:end);
        route.voieGx = route.voieGx(1:1:end);
        route.voieGy = route.voieGy(1:1:end);
        route.alpha = route.alpha(1:1:end);
        route.rayon = route.rayon(1:1:end);
        route.courbure = 1./route.rayon;
        route.courbure = route.courbure'; 
        route.step = route.distp;
        route.largeur = largeur;
        % detection Type de geometrie
        route.Type_geo = type_geometrie(route.courbure);
        EPHI0 = 0;
        route.EX0 = 0;
        route.EY0 = 0;
        
    case 5
        mat_file_path = '../SCANeR_matData/RL/Exp2_NoFog-No_Assistance';
        % Initialize a raw data adapter
        raw_data = MATDataAdapter(mat_file_path, 1);
        % Select data with speed > v_x_min and half of whole data
        v_x_min = 17;
        data = raw_data.select_indices((raw_data.speed_x >= v_x_min) & (raw_data.time <= max(raw_data.time)/2));
        
        road_center_x = data.road_center_x;
        road_center_x_pre = [road_center_x(1);road_center_x(1:end-1)];
        road_center_y = data.road_center_y;
        road_center_y_pre = [road_center_y(1);road_center_y(1:end-1)];
        diff = vecnorm([road_center_x - road_center_x_pre road_center_y - road_center_y_pre]');
        S = cumsum(diff);
        [S, indice] = unique(S);

%         road_center_x = road_center_x(indice);
%         road_center_y = road_center_y(indice);
%         vehicle_rear_axle_x = data.vehicle_rear_axle_center_x(indice);
%         vehicle_rear_axle_y = data.vehicle_rear_axle_center_y(indice);
%         vehicle_heading     = data.heading_rad(indice);
        
        route.step = 0.05;
        EPHI0=data.heading_rad(200); 
        route.EX0=data.vehicle_rear_axle_center_x(200);
        route.EY0=data.vehicle_rear_axle_center_y(200);
        route.largeur = 3.5;
        
        x2 = 0:0.05:S(end);
        ro_ref0 = interp1(S,data.road_curvature(indice),x2,'spline');
        road_angle = interp1(S, data.road_angle_rad(indice), x2, 'spline');
        centrex = interp1(S, data.road_center_x(indice), x2, 'spline');
        centrey = interp1(S, data.road_center_y(indice), x2, 'spline');
               
        route.rayon = -1./ro_ref0';
%         route = creeRouteRayonDeCourbure( route.rayon, route.step, route.EX0, route.EY0, EPHI0, route.largeur);
        route.courbure = ro_ref0';
        route.alpha = road_angle';
        
%         route.centrex = centrex';
%         route.centrey = centrey';
%         route.voieGx = route.centrex - route.largeur/2*sin(route.alpha);
%         route.voieGy = route.centrey + route.largeur/2*cos(route.alpha);
%         route.voieDx = route.centrex + route.largeur/2*sin(route.alpha);
%         route.voieDy = route.centrey - route.largeur/2*cos(route.alpha);

        route.voieGx = centrex';
        route.voieGy = centrey';
        route.centrex = route.voieGx + route.largeur/2*sin(route.alpha);
        route.centrey = route.voieGy - route.largeur/2*cos(route.alpha);
        route.voieDx =  route.voieGx + route.largeur*sin(route.alpha);
        route.voieDy = route.voieGy - route.largeur*cos(route.alpha);
       
    otherwise
    disp(' ');
    disp('Piste inconnue !!!');   
end


