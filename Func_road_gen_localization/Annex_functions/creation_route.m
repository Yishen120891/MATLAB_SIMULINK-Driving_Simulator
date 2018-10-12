%profil de courbure
% minimum vitesse 60Km/h= 16.7 m/s
% max Acceleration laterale= 4 m/s2 ==> Rmin=V^2_min/Accel_max=16.7^2/4=69.7m

circle=-1/70;%circle, virage à gauche
partie_sin=1/100;
step=0.1;
distance_chngement_R=100;

% virage 1= circle
u=zeros(5000,1);
u=[u;linspace(0,circle,distance_chngement_R/step)'];
u=[u;circle*ones(120/step,1)];
u=[u;linspace(circle,0,distance_chngement_R/step)'];
u=[u;zeros(100/step,1)];

% virage 2= Partie sinus

distance_changemnt_tan=150;
distance_virage=100;
for i=1:2
    u=[u;linspace(0,-partie_sin,distance_chngement_R/step)'];
    u=[u;linspace(-partie_sin,partie_sin,distance_changemnt_tan/step)'];
    u=[u;partie_sin*ones(distance_virage/step,1)];
    u=[u;linspace(partie_sin,-partie_sin,distance_changemnt_tan/step)'];
    u=[u;linspace(-partie_sin,0,distance_chngement_R/step)'];
end
u=[u;zeros(50/step,1)];

% virage 2= circle
u=[u;linspace(0,circle,distance_chngement_R/step)'];
u=[u;circle*ones(120/step,1)];
u=[u;linspace(circle,0,distance_chngement_R/step)'];
u=[u;zeros(500/step,1)];
%-----------

R = 1./u;
route=creeRouteRayonDeCourbure( R,step, 0,0,0, 3.3);
route.centrex = route.centrex(1:10:end);
route.centrey = route.centrey(1:10:end);
route.voieDx = route.voieDx(1:10:end);
route.voieDy = route.voieDy(1:10:end);
route.voieGx = route.voieGx(1:10:end);
route.voieGy = route.voieGy(1:10:end);
route.alpha = route.alpha(1:10:end);
route.rayon = route.rayon(1:10:end);

%  plot(route.voieDx,route.voieDy);
% grid on; hold on; 
% plot(route.voieGx,route.voieGy);
% plot(route.centrex,route.centrey,'r--');
% xlabel('X position [m]');ylabel('Y position [m]'); 
% 


% 
% %========================================
% % % profil de courbure
% umax1=0.02;
% umax2=0.03;
% umax3=0.05;
% umax4=0.072;
% umax5=0.06;
% %%%%% virage 1
% u=zeros(500,1);
% u=[u;linspace(0,umax1,400)'];
% u=[u;umax1*ones(1400,1)];
% u=[u;linspace(umax1,0,400)'];
% %%%%% virage 2
% u=[u;zeros(1500,1)];
% u=[u;linspace(0,-umax2,400)'];
% u=[u;-umax2*ones(1400,1)];
% u=[u;linspace(-umax2,0,400)'];
% %%%%% virage 3
% u=[u;zeros(2500,1)];
% u=[u;linspace(0,-umax3,400)'];
% u=[u;-umax3*ones(1400,1)];
% u=[u;linspace(-umax3,0,400)'];
% %%%%% virage 4
% u=[u;zeros(100,1)];
% u=[u;linspace(0,umax3,400)'];
% u=[u;umax3*ones(1400,1)];
% u=[u;linspace(umax3,0,400)'];
% %%%%% virage 5
% u=[u;zeros(3500,1)];
% u=[u;linspace(0,-umax4,400)'];
% u=[u;-umax4*ones(1400,1)];
% u=[u;linspace(-umax4,0,400)'];
% %%%%% virage 6
% u=[u;zeros(3500,1)];
% u=[u;linspace(0,-umax2,400)'];
% u=[u;-umax2*ones(1000,1)];
% u=[u;linspace(-umax2,0,400)'];
% %%%%% virage 7
% u=[u;zeros(200,1)];
% u=[u;linspace(0,-umax3,400)'];
% u=[u;-umax3*ones(500,1)];
% u=[u;linspace(-umax3,0,400)'];
% %%%%% virage 8
% u=[u;zeros(100,1)];
% u=[u;linspace(0,umax4,400)'];
% u=[u;umax4*ones(1000,1)];
% u=[u;linspace(umax4,0,400)'];
% %%%%% virage 9
% u=[u;zeros(1500,1)];
% u=[u;linspace(0,umax3,400)'];
% u=[u;umax3*ones(1400,1)];
% u=[u;linspace(umax3,0,400)'];
% u=[u;zeros(100,1)];
% %%%%% virage 10
% u=[u;zeros(500,1)];
% u=[u;linspace(0,-umax5,400)'];
% u=[u;-umax5*ones(1400,1)];
% u=[u;linspace(-umax5,0,400)'];
% u=[u;zeros(100,1)];
% %%%%% virage 11
% u=[u;zeros(100,1)];
% u=[u;linspace(0,umax1,200)'];
% u=[u;umax1*ones(1110,1)];
% u=[u;linspace(umax1,0,200)'];
% u=[u;zeros(2055,1)];
% u = u(1:2:end)';
% %%%%%%%%%%%%%%%%%%%%%%%%%%
% R = 1./u;
% Courb = u;
% route=creeRouteRayonDeCourbure( R,0.1, 0,0,pi/2, 3.3);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  route.centrex = route.centrex(1:10:end);
%  route.centrey = route.centrey(1:10:end);
%  route.voieDx = route.voieDx(1:10:end);
%  route.voieDy = route.voieDy(1:10:end);
%  route.voieGx = route.voieGx(1:10:end);
%  route.voieGy = route.voieGy(1:10:end);
%  route.alpha = route.alpha(1:10:end);
%  route.rayon = route.rayon(1:10:end);
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  posix =  route.centrex + 0.05*randn(1,length(route.centrex));
%  posiy =  route.centrey + 0.02*randn(1,length(route.centrey));
%  posix = posix(1:10:end);
%  posiy = posiy(1:10:end);
%  cap_route = route.alpha(1:10:end);
%  courbure = 1./route.rayon;
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  plot(route.voieDx,route.voieDy);
% grid on; hold on; 
% plot(route.voieGx,route.voieGy);
% plot(route.centrex,route.centrey,'r--');
% xlabel('X position [m]');ylabel('Y position [m]'); 