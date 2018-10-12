
function [A,B,C,D,K,x0]=driver_model(p,v,T,aux)
% Definition of the LTI state-space model of the driver
% See paper IEEE ITS Saleh et al. [SCCLM13]
%
%model 1: Anticipation : NOT USED
%-------------
% p=[Kp];
% A=0;
% B=0;
% C=0;
% D=p(1);
% K=0;
% x0=0;
%model 2: model teta with delay : NOT USED
%-------------
% p=[Kp      Kc        Ti       Tl    taup];
% p=[3       1         1        3     0.151];

% v=15;
% A=[-1/p(3) 0;
%    (p(2)/v)*(2/p(5))*(p(4)/p(3)-1)      -2/p(5)];
% B=[0 1/p(3);
%    2*p(1)/p(5)     -(p(2)/v)*(2/p(5))*p(4)/p(3) ];
% C=[-(p(2)/v)*(p(4)/p(3)-1)   2];
% D=[-p(1)       (p(2)/v)*p(4)/p(3)];
% K=[0;0];
% x0=[0;0];
%
%model4: USED ONE
%-------------
% p=[Kp      Kc      Ti      Tl     taup      Kr     Kt     Tn];
% p=[1        2      3       4       5        6      7      8]; % index

a11=-1/p(3);
a21= 2*(p(2)/v)*(p(4)/p(3)-1)/p(5);
a22=-2/p(5);
a31=-(p(2)/v)*(p(6)*v+p(7))*(p(4)/p(3)-1)/p(8);
a32=2*(p(6)*v+p(7))/p(8);
a33=-1/p(8);

b12=1/p(3);
b21=2*p(1)/p(5); 
b22=-2*(p(2)/v)*p(4)/(p(5)*p(3));
b31=-p(1)*(p(6)*v+p(7))/p(8); 
b32=(p(2)/v)*(p(6)*v+p(7))*p(4)/(p(3)*p(8)); 
b33=-p(7)/p(8);
b34=-1/p(8);

A=[a11 0     0;
   a21 a22   0;
   a31 a32  a33];

B=[ 0   b12 0   0;
    b21 b22 0   0;
    b31 b32 b33 b34];

C=[0    0   1];

D=[0    0   0   0];

% DEGUB TOOLS
% C=[0    0   1];
% D=[0    0   0   0];
%
% c21=-p(2)*(p(4)/p(3)-1)/v;
% d22=p(2)*p(4)/(p(3)*v);
% C=[0    0   1;
%    c21  2   0];
% D=[0    0   0   0;
%    -p(1) d22 0  0];


K=[0;0;0];
x0=[0;0;0];

%***********************************
% ctrb(A,[0;0;b34])
% if(rank(ctrb(A,[0;0;b34]))~=length(A))
%      error('uncontrolable system')
%      return
% end






