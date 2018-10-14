time = simut.RawData{1}.Values.Time;
rho = simut.RawData{1}.Values.Data;
Gamma_d = simut.RawData{2}.Values.Data;
delta_d = simut.RawData{3}.Values.Data;

fs = 1/(time(2)-time(1));

figure;
plot(time, rho, time, Gamma_d, time, delta_d);

IdenStartTime       = 20;
IdenEndTime         = 80;
ValidateStartTime   = 90;
ValidateEndTime     = 150;

indiceIdenStartTime = find(time >= IdenStartTime, 1);
indiceIdenEndTime = find(time >= IdenEndTime, 1);
indiceValidateDataStart = find(time >= ValidateStartTime, 1);
indiceValidateDataEnd = find(time >= ValidateEndTime, 1);

u_iden =    rho(indiceIdenStartTime:indiceIdenEndTime);
y_iden = [  Gamma_d(indiceIdenStartTime:indiceIdenEndTime)    ...
            delta_d(indiceIdenStartTime:indiceIdenEndTime)     ];
% 
u_valide =      rho(indiceValidateDataStart:indiceValidateDataEnd);
y_valide = [    Gamma_d(indiceValidateDataStart:indiceValidateDataEnd)    ...
                delta_d(indiceValidateDataStart:indiceValidateDataEnd)     ];
% 
data_iden = iddata(y_iden, u_iden, 1/fs);
data_valide = iddata(y_valide, u_valide, 1/fs);
% 
IdenOpt = ssestOptions;
    IdenOpt.Display = 'on';
    IdenOpt.EnforceStability = true;
    IdenOpt.InitialState = 'estimate';
    IdenOpt.OutputWeight = eye(2);
    IdenOpt.SearchOption.Tolerance = 1e-7;
    IdenOpt.SearchOption.MaxIter = 500;
    
IdenSys = ssest(data_iden, 1:10, 'Ts', data_iden.Ts, 'DisturbanceModel', 'none', IdenOpt);

figure;
compare(data_iden, IdenSys);
figure;
compare(data_valide, IdenSys);