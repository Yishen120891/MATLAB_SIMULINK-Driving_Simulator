%% Parameter estimation by PEM method
% 
%   Modify this file when:
%   - Initial values for PEM changed;
%   - PEM options changed.
%   - Validation methods and figures changed;
%
%                           Yishen Zhao
%                            15/10/2018

time        = simut.DriverModelRawData{1}.Values.Time;
Gamma_s     = simut.DriverModelRawData{1}.Values.Data;
delta_d     = simut.DriverModelRawData{2}.Values.Data;
theta_far   = simut.DriverModelRawData{3}.Values.Data;
theta_near  = -simut.DriverModelRawData{4}.Values.Data;
Gamma_d 	= simut.DriverModelRawData{5}.Values.Data;

u_measured  = [     theta_far'      ;...
                    theta_near'     ;...
                    delta_d'        ;...
                    Gamma_s'        ];
y_measured  = [     Gamma_d'        ]; 

fs = 1/(time(2)-time(1));

% figure;
% plot(time, rho, time, Gamma_d, time, delta_d);

IdenStartTime       = 20;
IdenEndTime         = 80;
ValidateStartTime   = 90;
ValidateEndTime     = 150;

indiceIdenStartTime     = find(time >= IdenStartTime, 1);
indiceIdenEndTime       = find(time >= IdenEndTime, 1);
indiceValidateDataStart = find(time >= ValidateStartTime, 1);
indiceValidateDataEnd   = find(time >= ValidateEndTime, 1);

% each signal needs to be stored as column vector
u_iden          = u_measured(:,indiceIdenStartTime:indiceIdenEndTime)';
y_iden          = y_measured(:,indiceIdenStartTime:indiceIdenEndTime)';

u_validate      = u_measured(:,indiceValidateDataStart:indiceValidateDataEnd)';
y_validate      = y_measured(:,indiceValidateDataStart:indiceValidateDataEnd)';

IdenData = iddata(y_iden, u_iden, 1/fs);

ValidateData = iddata(y_validate, u_validate, 1/fs);

%% Identification by PEM

% Nominal value and validation domain for parameters
K_p     = 3.4;              K_p_min = 2;                K_p_max = 5;
K_c     = 15;               K_c_min = 5;                K_c_max = 30;
T_I     = 1;                T_I_min = 0.5;              T_I_max = 2;
T_L     = 3;                T_L_min = 2;                T_L_max = 5;
tau_p   = 0.04;             tau_p_min = 0;              tau_p_max = 0.1;
K_r     = 1;                K_r_min = 0.5;              K_r_max = 1.5;
K_t     = 12;               K_t_min = 0;                K_t_max = 16;
T_N     = 0.1;
v       = 18;

InitialParameterValue = {'K_p', K_p; 'K_c', K_c; 'T_I', T_I; 'T_L', T_L; 'tau_p', tau_p; 'K_r', K_r; 'K_t', K_t; 'T_N', T_N; 'v', v};
OptionalArgumentsValue = { };

IdenModel = idgrey('funcDriverModelLoauy', InitialParameterValue, 'd', OptionalArgumentsValue, 1/fs);
% IdenModel = idgrey('funcDriverModelLoauy', InitialParameterValue, 'c', OptionalArgumentsValue, 0);
    IdenModel.Structure.Parameters(1).Minimum   = K_p_min;
    IdenModel.Structure.Parameters(1).Maximum   = K_p_max;
    IdenModel.Structure.Parameters(2).Minimum   = K_c_min;
    IdenModel.Structure.Parameters(2).Maximum   = K_c_max;
    IdenModel.Structure.Parameters(3).Minimum   = T_I_min;
    IdenModel.Structure.Parameters(3).Maximum   = T_I_max;
    IdenModel.Structure.Parameters(4).Minimum   = T_L_min;
    IdenModel.Structure.Parameters(4).Maximum   = T_L_max;
    IdenModel.Structure.Parameters(5).Minimum   = tau_p_min;
    IdenModel.Structure.Parameters(5).Maximum   = tau_p_max;
    IdenModel.Structure.Parameters(6).Minimum   = K_r_min;
    IdenModel.Structure.Parameters(6).Maximum   = K_r_max;
    IdenModel.Structure.Parameters(7).Minimum   = K_t_min;
    IdenModel.Structure.Parameters(7).Maximum   = K_t_max;
    IdenModel.Structure.Parameters(8).Free      = false;
    IdenModel.Structure.Parameters(9).Free      = false;

IdenOpt = greyestOptions;
    % Show estimation process
    IdenOpt.Display = 'on';
    % Disturbace model is not identified (no matrix K)
    IdenOpt.DisturbanceModel = 'none';
    % The initial state is treated as an independent estimation parameter.
    % Not very clear with this option, found in code of Ablamvi. (by Yishen, 05/10/2018)
    IdenOpt.InitialState = 'estimate';
    % Minimize trace(E'*E/N) instead of det(E'*E/N)
    IdenOpt.OutputWeight = eye(size(y_iden,1));
    % Maximum number of iterations during loss-function minimization
    IdenOpt.SearchOption.MaxIter = 500;
    % Minimum percentage difference between the current value of the loss function and its expected improvement after the next iteration
    IdenOpt.SearchOption.Tolerance = 1e-7;
    
IdenSys = greyest(IdenData, IdenModel, IdenOpt);

IdenParam = IdenSys.ParameterVector'
          
%% Validation of result
DriverSys = ss(IdenSys.A, IdenSys.B, IdenSys.C, IdenSys.D, 1/fs);
% DriverSys = ss(IdenSys.A, IdenSys.B, IdenSys.C, IdenSys.D);

figure;
compare(IdenData, DriverSys);

figure;
compare(ValidateData, DriverSys);