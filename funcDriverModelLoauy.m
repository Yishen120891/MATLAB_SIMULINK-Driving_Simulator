function [ A, B, C, D ] = funcDriverModelLoauy( K_p, K_c, T_I, T_L, tau_p, K_r, K_t, T_N, v, Ts )
%FUNCDRIVERMODELLOAUY ODE function of driver model in Louay Saleh's thesis used for PEM parameter
%estimation and calculation of matrice A, B, C, D. 
%   The driver model is described in state representation with:
%   System inputs:      [Theta_far Theta_near delta_d Gamma_s]
%   System outputs:     [Gamma_d]
%   System parameters:  [K_p K_c T_I T_L tau_p K_r K_t T_N v]
% -------------------------------------------------------------------------------
%   Inputs:
%       K_p:        visual anticipation gain
%       K_c:        visual compensation gain
%       T_I:        compensation time constant (band)
%       T_L:        compensation time constant (friction)
%       tau_p:      processing delay   
%       K_r:        steering column stiffness internal gain
%       K_t:        stretch reflex gain
%       T_N:        neuromuscular time constant
%       v:          vehicle velocity
%       Ts:         sample time
%           
%
%   Outputs:
%       [ A, B, C, D ]: system state representation matrix
%
%   Dicretization method: 
%       Euler
% -------------------------------------------------------------------------------
%   Orignial code of Ablamvi, modified by Yishen on 26/09/2018
%   Orignial function: [ A, B, C, D, K, x0 ] = Driver_Model_Sans_TL( param, T, delta_t )
%   K and x0 are not necessary in my case.
%   Pay attention to the order of inputs and outputs in identification data
%
%   Additional Remark from Ablamvi: retard:padé d'ordre 1

%% Model

A       = zeros(3,3);
B       = zeros(3,4);
C       = zeros(2,3);
D       = zeros(2,4);

A(1,1)  = -1/T_I;
A(2,1)  = -2*(K_c/v)*(1/tau_p)*(T_L/T_I -1);
A(2,2)  = -2*(1/tau_p);
A(3,1)  = (K_c/v)*(K_r*v+K_t)*(1/T_N)*(T_L/T_I-1);
A(3,2)  = 2*(K_r*v+K_t)*(1/T_N);
A(3,3)  = -1/T_N;

B(1,2)  = 1/T_I;
B(2,1)  = 2*K_p*(1/tau_p);
B(2,2)  = 2*(K_c/v)*(1/tau_p)*(T_L/T_I);
B(3,1)  = -K_p*(K_r*v+K_t)*(1/T_N);
B(3,2)  = -(K_c/v)*(K_r*v+K_t)*(1/T_N)*(T_L/T_I);
B(3,3)  = -K_t*(1/T_N);
B(3,4)  = -1/T_N;

C(1,3)  = 1;
C(2,1)  = K_c/v*(T_L/T_I-1);
C(2,2)  = 2;

D(2,1)  = -K_p;
D(2,2)  = -K_c/v*T_L/T_I;

% % Discretization
if Ts > 0
    A       = eye(3) + A*Ts;
    B       = B*Ts;
end

% P = inv(2/Ts*eye(3)-A);
% A = (2/Ts*eye(3)+A)*P;
% B = 2/Ts*P*B;

end

