
function [A,B1,B2,C,D,C2t,D22t]=system_driver_car(p,mu,M,v,lw,J,cf0,cr0,lf,lr,ls,Kp,nt,Rs,Is,Bs,Dfar,alpha2)


%car
%---
cf=mu*cf0;
cr=mu*cr0;
a11c=-2*(cf+cr)/(M*v);
a12c=-1+2*(lr*cr-lf*cf)/(M*v^2);
a15c=2*cf/(M*v*Rs);
a21c=2*(lr*cr-lf*cf)/J;
a22c=-2*(lr^2*cr+lf^2*cf)/(J*v);
a25c=2*(cf*lf)/(J*Rs);

Ts_beta=2*Kp*cf*nt/Rs;
Ts_r=2*Kp*cf*lf*nt/(Rs*v);

a61c=Ts_beta/Is;
a62c=Ts_r/Is;
a65c=-Ts_beta/(Rs*Is);
a66c=-Bs/Is;
b61c=1/Is;

%driver
%-------
a11d=-1/p(3);
a21d= 2*(p(2)/v)*(p(4)/p(3)-1)/p(5);
a22d=-2/p(5);
a31d=-(p(2)/v)*(p(6)*v+p(7))*(p(4)/p(3)-1)/p(8);
a32d=2*(p(6)*v+p(7))/p(8);
a33d=-1/p(8);

b12d=1/p(3);
b21d=2*p(1)/p(5); 
b22d=-2*(p(2)/v)*p(4)/(p(5)*p(3));
b31d=-p(1)*(p(6)*v+p(7))/p(8); 
b32d=(p(2)/v)*(p(6)*v+p(7))*p(4)/(p(3)*p(8)); 
b33d=-p(7)/p(8);
b34d=-1/p(8);
bn31d=-b34d*Ts_beta;
bn32d=-b34d*Ts_beta*lf/v;
bn35d=b33d+Ts_beta*b34d/Rs;
%--------------



A=[a11c  a12c   0    0       a15c   0    0    0    0;
   a21c  a22c   0    0       a25c   0    0    0    0;
   0     1      0    0       0      0    0    0    0;
   v     ls     v    0       0      0    0    0    0;
   0     0      0    0       0      1    0    0    0;
   a61c  a62c   0    0       a65c   a66c 0    0    b61c;
   0     0      b12d b12d/ls 0      0    a11d 0    0;
   0     0      b22d b22d/ls 0      0    a21d a22d 0;
   bn31d bn32d  b32d b32d/ls bn35d  0    a31d a32d a33d];

B1=[0;
    0;
    0;
    0;
    0;
    b61c;
    0;
    0;
    -b34d];%

B2=[0;
    0;
    -v;
    -v*ls;
    0;
    0;
    0;
    b21d*Dfar;
    b31d*Dfar];

%alpha=4;   %pourcentahe=1/(1+1/alpha)=80%
%alpha=7/3;   %pourcentahe=1/(1+1/alpha)=70%
%alpha=3/2;   %pourcentahe=1/(1+1/alpha)=60%
   %pourcentahe=1/(1+1/alpha)=50%
%alpha=2/3; %pourcentahe=1/(1+1/alpha)=40%
% alpha=3/7; %pourcentahe=1/(1+1/alpha)=30%
% alpha=1/4; %pourcentahe=1/(1+1/alpha)=20%
%alpha=1/9; %pourcentahe=1/(1+1/alpha)=10%
%alpha=1/99; %pourcentahe=1/(1+1/alpha)=20%
%alpha2 = 3/2;


C=[0      0      1   0   0       0   0   0   0;
   0      0      -ls 1   0       0   0   0   0;
   v*a11c v*a12c 0   0   v*a15c  0   0   0   0;
   0      0      0   0   0       0   0   0   -alpha2;
   0      0      0   0   0       0   0   0   1;
   0      0      0   0   0       0   0   0   0];
D=[0;
   0;
   0;
   1;
   0;
   1];

C2t=[1  0   0   0   0   0   0   0   0;
    0   1   0   0   0   0   0   0   0;
    0   0   1   0   0   0   0   0   0;
    0   0   0   1   0   0   0   0   0;
    0   0   0   0   1   0   0   0   0;
    0   0   0   0   0   1   0   0   0;
    0   0   0   0   0   0   0   0   1];
D22t=[1.5;3.4;2.5; 6.3; 9.8; 7.8; 3.8]; %D22t*D22t'=1