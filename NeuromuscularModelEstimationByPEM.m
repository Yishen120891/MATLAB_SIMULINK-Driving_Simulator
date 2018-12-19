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
u_iden          = [y_measured(2,indiceIdenStartTime:indiceIdenEndTime)' u_measured(3:4,indiceIdenStartTime:indiceIdenEndTime)'];
y_iden          = y_measured(1,indiceIdenStartTime:indiceIdenEndTime)';

u_validate      = [y_measured(2,indiceValidateDataStart:indiceValidateDataEnd)' u_measured(3:4,indiceValidateDataStart:indiceValidateDataEnd)'];
y_validate      = y_measured(1,indiceValidateDataStart:indiceValidateDataEnd)';

IdenData = iddata(y_iden, u_iden, 1/fs);
%     IdenData.InputName = u_names;
%     IdenData.OutputName = y_names;
    IdenData.Tstart     = IdenStartTime;

ValidateData = iddata(y_validate, u_validate, 1/fs);
%     ValidateData.InputName = u_names;
%     ValidateData.OutputName = y_names;

%% Identification by PEM

% Nominal value and validation domain for parameters
K_r     = 1;                K_r_min = 0.5;              K_r_max = 1.5;
K_t     = 12;               K_t_min = 0;                K_t_max = 16;
T_N     = 0.1;
v       = 17;

InitialParameterValue = {'K_r', K_r; 'K_t', K_t; 'T_N', T_N; 'v', v};
OptionalArgumentsValue = { };

% IdenModel = idgrey('funcDriverModelLoauy', InitialParameterValue, 'd', OptionalArgumentsValue, 1/fs);
IdenModel = idgrey('funcNeuromuscularModel', InitialParameterValue, 'c', OptionalArgumentsValue, 0);

    IdenModel.Structure.Parameters(1).Info.Label = 'K_r';
%     IdenModel.Structure.Parameters(1).Minimum   = K_r_min;
%     IdenModel.Structure.Parameters(1).Maximum   = K_r_max;
%     IdenModel.Structure.Parameters(1).Free      = false;   

    IdenModel.Structure.Parameters(2).Info.Label = 'K_t';
%     IdenModel.Structure.Parameters(2).Minimum   = K_t_min;
%     IdenModel.Structure.Parameters(2).Maximum   = K_t_max; 
%     IdenModel.Structure.Parameters(2).Free      = false;

    IdenModel.Structure.Parameters(3).Info.Label = 'T_N';
%     IdenModel.Structure.Parameters(3).Free      = false;

    IdenModel.Structure.Parameters(4).Info.Label = 'v';
    IdenModel.Structure.Parameters(4).Free      = false;

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
    IdenOpt.SearchOption.MaxIter = 500;
    % Minimum percentage difference between the current value of the loss function and its expected improvement after the next iteration
    IdenOpt.SearchOption.Tolerance = 1e-20;
    
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