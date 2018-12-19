function [ A, B, C, D ] = funcTwoPointsVisualModel( K_p, K_c, T_I, T_L, tau_p, v, Ts )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    A = zeros(2,2);
    B = zeros(2,2);
    C = zeros(1,2);
    D = zeros(1,2);
    
    A(1,1) = -1/T_I;
    A(2,1) = (2/tau_p)*(K_c/v)*(1-T_L/T_I);
    A(2,2) = -2/tau_p;
    
    B(1,2) = 1/T_I;
    B(2,1) = 2*K_p*(1/tau_p);
    B(2,2) = 2*(K_c/v)*(1/tau_p)*(T_L/T_I);
    
    C(1,1) = K_c/v*(T_L/T_I-1);
    C(1,2) = 2;
    
    D(1,1) = -K_p;
    D(1,2) = -K_c/v*T_L/T_I;
    
    % % Discretization
    if Ts > 0
        A = eye(3) + A*Ts;
        B = B*Ts;
    end
end

