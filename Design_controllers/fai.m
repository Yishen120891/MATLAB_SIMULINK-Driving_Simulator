function fai_t=fai(Aplus,Pplus,T,R,B1,B2,t)
fai_t= -inv(R)*B1'*expm(Aplus'*(T-t))*Pplus*B2;
