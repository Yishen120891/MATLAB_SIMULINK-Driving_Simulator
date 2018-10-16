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
%   Additional Remark from Ablamvi: retard:pad� d'ordre 1

%% Model

a11 = -1/T_I;
a12 = 0;
a13 = 0;
a21 = -2*(K_c/v)*(1/tau_p)*(T_L/T_I -1);
a22 = -2*(1/tau_p);
a23 = 0;
a31 = (K_c/v)*(K_r*v+K_t)*(1/T_N)*(T_L/T_I-1);
a32 = 2*(K_r*v+K_t)*(1/T_N);
a33 = -1/T_N;

b11 = 0;
b12 = 1/T_I;
b13 = 0;
b14 = 0;
b21 = 2*K_p*(1/tau_p);
b22 = 2*(K_c/v)*(1/tau_p)*(T_L/T_I);
b23 = 0;
b24 = 0;
b31 = -K_p*(K_r*v+K_t)*(1/T_N);
b32 = -(K_c/v)*(K_r*v+K_t)*(1/T_N)*(T_L/T_I);
b33 = -K_t*(1/T_N);
b34 = -1/T_N;

A = [       a11     a12     a13     ;...
            a21     a22     a23     ;...
            a31     a32     a33     ];

B = [       b11     b12     b13     b14     ;...
            b21     b22     b23     b24     ;...
            b31     b32     b33     b34     ];

% C = [               0               0               1;...
%                     0               1               0               ];
C = [               0               0               1               ];

% D = [               0               0               0               0;
%                     0               0               0               0           ];

D = [               0               0               0               0];
                
% Discretization
A = eye(3) + A*Ts;
B = B*Ts;

end

