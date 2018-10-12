
function [Kplus, fai_0, fai_T8, fai_2T8,fai_3T8,fai_4T8,fai_5T8,fai_6T8,fai_7T8,fai_T,F2]=H2preview(A,B1,B2,R,Q,S,T,Aw,Cw)

% Riccati Equation: Pplus*A+A'*Pplus-(S+Pplus*B1)*inv(R)*(S'+B1'*Pplus)+Q=0
[Pplus,Cloop_eigens,Kplus,report]= care(A,B1,Q,R,S);
% Kplus= inv(R)*B1'*Pplus;
Kplus;
Aplus=A-B1*inv(R)*(S'+B1'*Pplus);
eg=eig(Aplus);

% fai_0  = fai(Aplus,Pplus,T,R,B1,B2,0)
% fai_T3 = fai(Aplus,Pplus,T,R,B1,B2,T/3)
% fai_2T3=fai(Aplus,Pplus,T,R,B1,B2,2*T/3)
% fai_T  = fai(Aplus,Pplus,T,R,B1,B2,T)

fai_0  = fai(Aplus,Pplus,T,R,B1,B2,0);
fai_T8 = fai(Aplus,Pplus,T,R,B1,B2,T/8);
fai_2T8 = fai(Aplus,Pplus,T,R,B1,B2,2*T/8);
fai_3T8 = fai(Aplus,Pplus,T,R,B1,B2,3*T/8);
fai_4T8 = fai(Aplus,Pplus,T,R,B1,B2,4*T/8);
fai_5T8 = fai(Aplus,Pplus,T,R,B1,B2,5*T/8);
fai_6T8 = fai(Aplus,Pplus,T,R,B1,B2,6*T/8);
fai_7T8 = fai(Aplus,Pplus,T,R,B1,B2,7*T/8);
fai_T  = fai(Aplus,Pplus,T,R,B1,B2,T);


% F2 selon general
 M=lyapkr(Aplus',Aw,Pplus*B2*Cw);
 F2=-inv(R)*B1'*expm(Aplus'*T)*M;


