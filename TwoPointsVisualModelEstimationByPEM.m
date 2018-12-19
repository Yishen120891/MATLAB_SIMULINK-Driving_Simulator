%% Parameter estimation by PEM method
% 
%   Modify this file when:
%   - Initial values for PEM changed;
%   - PEM options changed.
%   - Validation methods and figures changed;
% 
%   Variables from RunMe.m :
%   - IdenStartTime
%   - IdenEndTime
%   - ValidateStartTime
%   - ValidateEndTime
%
%   Variables from IdentificationDataCalculation.m :
%   - u_measured
%   - y_measured
%   - fs
%
%                           Yishen Zhao
%                            05/10/2018

fprintf('===================== ParameterEstimationByPEM =====================\n\n');

IdenStartTime = 60;
IdenEndTime = 120;
ValidateStartTime = 120;
ValidateEndTime = 180;

time = simut.temps;
u_measured = [simut.theta_far -simut.theta_near simut.deltad simut.Ts]';
y_measured = [simut.Td simut.delta_SW]';
fs = 1/(time(2)-time(1));

indiceIdenStartTime     = find(time >= IdenStartTime, 1);
indiceIdenEndTime       = find(time >= IdenEndTime, 1);
indiceValidateDataStart = find(time >= ValidateStartTime, 1);
indiceValidateDataEnd   = find(time >= ValidateEndTime, 1);

% each signal needs to be stored as column vector
t_iden          = time(indiceIdenStartTime:indiceIdenEndTime,:);
u_iden          = u_measured(1:2,indiceIdenStartTime:indiceIdenEndTime)';
y_iden          = y_measured(2,indiceIdenStartTime:indiceIdenEndTime)';

u_validate      = u_measured(1:2,indiceValidateDataStart:indiceValidateDataEnd)';
y_validate      = y_measured(2,indiceValidateDataStart:indiceValidateDataEnd)';

IdenData = iddata(y_iden, u_iden, 1/fs);
%     IdenData.InputName = u_names(1:2);
%     IdenData.OutputName = y_names(1);
    IdenData.Tstart     = IdenStartTime;

ValidateData = iddata(y_validate, u_validate, 1/fs);
%     ValidateData.InputName = u_names(1:2);
%     ValidateData.OutputName = y_names(1:2);

%% Identification by PEM

% Nominal value and validation domain for parameters
K_p     = 3.4;              K_p_min = 2;                K_p_max = 5;
K_c     = 15;               K_c_min = 5;                K_c_max = 30;
T_I     = 1;                T_I_min = 0.5;              T_I_max = 2;
T_L     = 3;                T_L_min = 2;                T_L_max = 5;
tau_p   = 0.04;             tau_p_min = 0;              tau_p_max = 0.1;
v       = 17;

InitialParameterValue = {'K_p', 1; 'K_c', 1; 'T_I', 1; 'T_L', 1; 'tau_p', 1; 'v', v};
OptionalArgumentsValue = { };

% IdenModel = idgrey('funcDriverModelLoauy', InitialParameterValue, 'd', OptionalArgumentsValue, 1/fs);
IdenModel = idgrey('funcTwoPointsVisualModel', InitialParameterValue, 'c', OptionalArgumentsValue, 0);
    IdenModel.Structure.Parameters(1).Info.Label = 'K_p';
%     IdenModel.Structure.Parameters(1).Minimum   = K_p_min;
%     IdenModel.Structure.Parameters(1).Maximum   = K_p_max;
%     IdenModel.Structure.Parameters(1).Free      = false;

    IdenModel.Structure.Parameters(2).Info.Label = 'K_c';
%     IdenModel.Structure.Parameters(2).Minimum   = K_c_min;
%     IdenModel.Structure.Parameters(2).Maximum   = K_c_max;
%     IdenModel.Structure.Parameters(2).Free      = false;

    IdenModel.Structure.Parameters(3).Info.Label = 'T_I';
%     IdenModel.Structure.Parameters(3).Minimum   = T_I_min;
%     IdenModel.Structure.Parameters(3).Maximum   = T_I_max;
%     IdenModel.Structure.Parameters(3).Free      = false;

    IdenModel.Structure.Parameters(4).Info.Label = 'T_L';    
%     IdenModel.Structure.Parameters(4).Minimum   = T_L_min;
%     IdenModel.Structure.Parameters(4).Maximum   = T_L_max;
%     IdenModel.Structure.Parameters(4).Free      = false;

    IdenModel.Structure.Parameters(5).Info.Label = 'tau_p';
%     IdenModel.Structure.Parameters(5).Minimum   = tau_p_min;
%     IdenModel.Structure.Parameters(5).Maximum   = tau_p_max;
%     IdenModel.Structure.Parameters(5).Free      = false;

    IdenModel.Structure.Parameters(6).Info.Label = 'v';
    IdenModel.Structure.Parameters(6).Free      = false;

IdenOpt = greyestOptions;
    % Show estimation process
    IdenOpt.Display = 'on';
    % Disturbace model is not identified (no matrix K)
    IdenOpt.DisturbanceModel = 'none';
    % Enforce stability
    IdenOpt.EnforceStability = true;
    % The initial state is treated as an independent estimation parameter.
    % Not very clear with this option, found in code of Ablamvi. (by Yishen, 05/10/2018)
    IdenOpt.InitialState = 'estimate';
    % Minimize trace(E'*E/N) instead of det(E'*E/N)
    IdenOpt.OutputWeight = eye(size(y_iden,1));
    % Maximum number of iterations during loss-function minimization
    IdenOpt.SearchOption.MaxIter = 200;
    % Minimum percentage difference between the current value of the loss function and its expected improvement after the next iteration
    IdenOpt.SearchOption.Tolerance = 1e-7;
    
IdenSys = greyest(IdenData, IdenModel, IdenOpt);

IdenSys.Report.Fit
IdenSys.Report.Termination
IdenParam = getpvec(IdenSys,'free')'
IdenParamCov = getcov(IdenSys, 'value', 'free')
          
%% Validation of result
figure;
compare(IdenData, IdenSys);

figure;
compare(ValidateData, IdenSys);

fprintf('====================================================================\n');