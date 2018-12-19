function [ A, B, C, D ] = funcNeuromuscularModel( K_r, K_t, T_N, v, Ts )
%FUNCNEUROMUSCULARMODEL Summary of this function goes here
%   Detailed explanation goes here

    A = zeros(1,1);
    B = zeros(1,3);
    C = zeros(1,1);
    D = zeros(1,3);

    A(1,1) = -1/T_N;
    B(1,1) = 1/T_N*(K_r*v+K_t);
    B(1,2) = -1/T_N*K_t;
    B(1,3) = -1/T_N;
    C(1,1) = 1;

    % % Discretization
    if Ts > 0
        A = eye(3) + A*Ts;
        B = B*Ts;
    end

end

