
function [Abic,Bbic,Cbic,Dbic,A,B1,B2,C,D1,D2]= Vehicle_Combined(mu,M,v,lw,J,cf0,cr0,lf,lr,ls,Kp,nt,Rs,Is,Bs)
% le modèle combiné: bicyclette + positionnement + colonne de direction
    % [A,Bc,Bp,C]=Vehicle_Combined(0.8,1500,10,0.4,1454,40000,35000,1,1.5,5,1,0.13,21,1,10)
    % Proposition of typical numerical values hereby
    %
    %---------------------------------------------
    %     dx/dt= A.x + B1.(Ta+Td) + B2.[fw roref]'
    %     y    = C.x + D1.(Ta+Td) + D2.[fw roref]' 
    %     x=[beta; r; PsiL; Yl, delta_d, delta_dp]
    %     y=      [PsiL; yact, a,(Ta+Td)] 
    %
    % Remark :  a = lateral acceleration of the vehicle
    %          Ta = Assistance torque (if any)
    %          Td = Driver torque (if any)
    %---------------------------------------------
    
    % Définition de la matrice A
    cf=mu*cf0;
    cr=mu*cr0;
    a11=-2*(cf+cr)/(M*v);
    a12=-1+2*(lr*cr-lf*cf)/(M*v^2);
    a21=2*(lr*cr-lf*cf)/J;
    a22=-2*(lr^2*cr+lf^2*cf)/(J*v);
    Ts_beta=2*Kp*cf*nt/Rs;
    Ts_r=2*Kp*cf*lf*nt/(Rs*v);
    b1=2*cf/(M*v);
    b2=2*(cf*lf)/J;
    
    A=[        a11     a12   0  0           b1/Rs       0;
               a21     a22   0  0           b2/Rs       0;
                 0       1   0  0               0       0;
                 v      ls   v  0               0       0;
                 0       0   0  0               0       1;
        Ts_beta/Is Ts_r/Is   0  0 -Ts_beta/(Rs*Is) -Bs/Is];
    
    % Définition de la matrice B1 
    B1=[   0;
           0;
           0;
           0;
           0;
        1/Is];
    
    % Définition de la matrice B2
    % input signals[fw roref]'
    e11=1/(M*v);
    e22=lw/J;
    B2=[e11       0;
        e22       0;
          0      -v;
          0   -v*ls;
          0       0;
          0       0];
   
    % Définition de la matrice C
    %         C=[ 0 1 0 0 0 0;
    %             0 0 1 0 0 0
    %             0 0 0 1 0 0
    %             0 0 0 0 1 0
    %             0 0 0 0 0 1];

    C=[     0     0   1  0       0  0;  % psiL
            0     0 -ls  1       0  0;  %yact
        a11*v a12*v   0  0 b1*v/Rs  0;  % a
            0     0   0  0       0  0]; % Tv = Ta+Td

    % Définition de la matrice D1    
    D1=[0;
        0;
        0;
        1];
    
    % Définition de la matrice D2
    D2=[0;
        0;
     -v^2;
        0];

 %---------------------------------------------
    % modèle bicyclette
    %     dx/dt= A.x + B [deltad fw roref]'
    %     y    = C.x + D.[deltad fw roref]'
    %     x=[beta; r; PsiL; yl]
    %     y=[beta r psiL yL Ts]  
    %
    % Remark: Ts = self-aligning torque of the front wheels
    % see e.g. Sentouh et al., DSC [SCCM09]
    %
    %---------------------------------------------
    % Définition de la matrice A
    Abic=[a11 a12   0  0;
          a21 a22   0  0;
            0   1   0  0;
            v  ls   v  0];
    
    % Définition de la matrice B 
    Bbic=[b1/Rs  e11     0;
          b2/Rs  e22     0;
              0    0    -v;
              0    0 -v*ls]; 
       
    
    % Définition de la matrice C
    Cbic=[    1         0       0    0;
              0         1       0    0;
              0         0       1    0;
              0         0       0    1;
       -Ts_beta     -Ts_r       0    0];
              
    % Définition de la matrice D
    Dbic=[                0   0   0;
                          0   0   0;
                          0   0   0;
                          0   0   0;
          2*Kp*cf*nt/(Rs^2)   0   0 ];
              
