

Test_route.indice=[0:1:5000];

x=0:0.5:600;
y=300+sqrt(90000-(x-300).^2);
plot(x,y)

(circle.x-300)+=

%une zone rectangulaire de X=1600m et Y=600m
%on construit un piste avec resolution 0.5m
X=1600;
Y=600;
res=0.5;
Xgraph=0:res:X;
Ygraph=0:res:Y;
% la voiture conmmence sa course à partir de (x0,y0)=(800,100);
x0=800;
y0=100;
x=Xgraph-x0;
y=Ygraph-y0;

clear all;
res=0.5;
for i=1:800
    Test_route.x(i)=800+i*res;
    Test_route.y(i)=100;
end

for i=801:2656 %801+pi*300/res 
   Test_route.y(i)=100+(i-800)*res;
   Test_route.x(i)=1200+sqrt(90000-(Test_route.y(i)-400).^2);
end

for i=1744:1743+800/res
    Test_route.x(i)=Test_route.x(1743)-(i-1743)*res;
    Test_route.y(i)=700;
end

plot(Test_route.x,Test_route.y)
    
    
%un segment ligne de longeur 800m entre (400,100) et (1200,100)
segment_ligne= 
F=0.2;
w=2*pi*F;
segment_sin=200 * sin (w * [0:0.5:800]);
plot(segment_sin);


t=0:0.5:100;
w=0.2;
f=200*sin(w*t);
plot(f);

aux1=-w^2*sin(w*t);
aux2=(1+w^2*(cos(w*t)).^2).^1.5;
ro=aux1./aux2;
plot(ro);
