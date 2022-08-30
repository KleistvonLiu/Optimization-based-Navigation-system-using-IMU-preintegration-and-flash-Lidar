%% === IMU unit

%先加载mat 包含了acc gyro，然后在slx里面设置一下运行时间 tEnd

% --- Acc. true measurements
%  -- inertial acceleration

%acc.accTrue = dataBase.IMUmeas.acc_IB_B_ref;

% flags to set measurement noises
acc.flagWhiteNoise = 1;% 1 or 0
acc.flagBiasInstability = 1;


% --- Gyro. true measurements

% -- inertial angular rates

%gyro.angRateTrue = dataBase.IMUmeas.angRate_IB_B_ref;

% refAngRate_BL_B = rad2deg(sl.EnvModel.satRateBL_B');

% flags to set measurement noises
gyro.flagWhiteNoise = 1;
gyro.flagBiasInstability = 1;


% --- configuration of IMU measurement noises
INS_UT = dtSim;  % s, INS (U)pdate (T)ime
IMU_OUT_UT = INS_UT; % s, IMU (OUT)put (U)pdate (T)ime
noiseSource_Ts = 0.01*1/((1/IMU_OUT_UT)/4); % 1.test: 0.0004s, 2.test: 0.004s

% --- Acc. measurement noises
% defAccA.N   = 4.2e-4 * 9.81/sqrt(3600);      % velocity random walk (velocity random walk coefficient)
% defAccA.K   = 5e-6;						    % linear acceleration random walk  (linear acceleration random walk coefficient)
% defAccA.B   = 4e-6 * 9.81/3600;             	% bias instability  (flicker acceleration coefficient)
% eps = 0.01;                                     % accuracy of flicker acceleration state-space model        [-]
% numStates = 20;                                 % number of required flicker acceleration system states     [-]

%  -- accelerometer white noise
acc_N = 4.2e-4 * 9.81/sqrt(3600); % [m/s^(3/2)]
whiteNoiseAcc.PSD = acc_N^2;%0.00000023361; % [(m/s^2)^2/Hz] PSD of white noise
whiteNoiseAcc.Ts = noiseSource_Ts;% [s] sample time of white noise
seedNumAcc = floor(rand(3,1)*1000)%0,150,100,500

%  -- accelerometer bias instability
% bias instability coefficient B - parameter of representative bias instability mathematical model
% for x axis
param_accx_bi.B       = 4e-6 * 9.81/3600; % [m/s^2]
param_accx_bi.p       = 5;
tau_left              = 10;      % [s] % left boundary of the bottom plateau 
tau_right             = 1000;    % [s] % right boundary of the bottom plateau
param_accx_bi.tau_max = calc_tau_max_bi_pth_order(tau_left, tau_right, param_accx_bi.p);
% other parameters
param_accx_bi.T_B	  = param_accx_bi.tau_max / 1.89; % [s]

param_accx_bi.S_n     = 2*param_accx_bi.B^2 * log(2) ./ (pi*0.4365^2*param_accx_bi.T_B);
f_B                   = 1./(2*pi*param_accx_bi.T_B);% [Hz]
Ts_temp               = (1/100) * (1/max(f_B)); % [s] - scalar!
param_accx_bi.Ts      = floor(Ts_temp / noiseSource_Ts) * noiseSource_Ts;
param_accx_bi.mean_rg = zeros(param_accx_bi.p,1);
param_accx_bi.var_rg  = param_accx_bi.S_n / param_accx_bi.Ts; % [(°/s)^2/s]
param_accx_bi.seed    = (1:param_accx_bi.p)'*23235; % must be different!

param_accx_bi.Am = eye(param_accx_bi.p,param_accx_bi.p) .* repmat((-1./param_accx_bi.T_B),1,param_accx_bi.p);
param_accx_bi.Bm = eye(param_accx_bi.p,param_accx_bi.p);
param_accx_bi.Cm = ones(1,param_accx_bi.p);
param_accx_bi.Dm = zeros(1,param_accx_bi.p);

%   - for y and z axis
param_accy_bi = param_accx_bi;
param_accy_bi.seed    = (1:param_accx_bi.p)'*24562; % must be different!
param_accz_bi = param_accx_bi;
param_accz_bi.seed    = (1:param_accx_bi.p)'*13254; % must be different!


% --- Gyro. measurement noise parameterization
% defGyroA.N   = 0.0002 * pi/180/sqrt(3600);       % angle random walk (random walk coefficient)       [rad/sqrt(s)]
% defGyroA.K   = 3.5e-4 * pi/180/sqrt(3600^3);     % rate random walk  (rate random walk coefficient)  [rad/sqrt(s3)]
% defGyroA.B   = 5e-4/3 * pi/180/3600;             % bias instability  (flicker rate coefficient)      [rad/s]
% eps = 0.01;                                     % accuracy of flicker rate state-space model        [-]
% numStates = 20;                                 % number of required flicker rate system states     [-]

%  -- gyroscope white noise
gyro_N = 0.0002 * pi/180/sqrt(3600); % [rad/s^(1/2)]
whiteNoiseGyro.PSD = gyro_N^2;%0.000025; % [(deg/s)^2/Hz] PSD of white noise
whiteNoiseGyro.Ts = noiseSource_Ts;% [s] sample time of white noise
seedNumGyro = floor(rand(3,1)*1000)%0,20,200,800

%  -- gyroscope bias instability
% bias instability coefficient B - parameter of representative bias instability mathematical model
% for x axis
param_gyrox_bi.B       = 5e-4/3 * pi/180/3600; % [rad/s]
param_gyrox_bi.p       = 5;
tau_left              = 10;      % [s] % left boundary of the bottom plateau 
tau_right             = 1000;    % [s] % right boundary of the bottom plateau
param_gyrox_bi.tau_max = calc_tau_max_bi_pth_order(tau_left, tau_right, param_gyrox_bi.p);
% other parameters
param_gyrox_bi.T_B	  = param_gyrox_bi.tau_max / 1.89; % [s]

param_gyrox_bi.S_n     = 2*param_gyrox_bi.B^2 * log(2) ./ (pi*0.4365^2*param_gyrox_bi.T_B);
f_B                   = 1./(2*pi*param_gyrox_bi.T_B);% [Hz]
Ts_temp               = (1/100) * (1/max(f_B)); % [s] - scalar!
param_gyrox_bi.Ts      = floor(Ts_temp / noiseSource_Ts) * noiseSource_Ts;
param_gyrox_bi.mean_rg = zeros(param_gyrox_bi.p,1);
param_gyrox_bi.var_rg  = param_gyrox_bi.S_n / param_gyrox_bi.Ts; % [(°/s)^2/s]
param_gyrox_bi.seed    = (1:param_gyrox_bi.p)'*2354; % must be different!

param_gyrox_bi.Am = eye(param_gyrox_bi.p,param_gyrox_bi.p) .* repmat((-1./param_gyrox_bi.T_B),1,param_gyrox_bi.p);
param_gyrox_bi.Bm = eye(param_gyrox_bi.p,param_gyrox_bi.p);
param_gyrox_bi.Cm = ones(1,param_gyrox_bi.p);
param_gyrox_bi.Dm = zeros(1,param_gyrox_bi.p);

%   - for y and z axis
param_gyroy_bi = param_gyrox_bi;
param_gyroy_bi.seed    = (1:param_gyrox_bi.p)'*2859; % must be different!
param_gyroz_bi = param_gyrox_bi;
param_gyroz_bi.seed    = (1:param_gyrox_bi.p)'*3491; % must be different!



% --- generate IMU measurement noises

%  -- Low pass/antialiasing filter configuration
INS_UF = 1/INS_UT; % [Hz] INS update frequency
LPF.PT2.fB = INS_UF / 4; % upper frequency of -3dB bandwidth of PT2 [Hz]
LPF.PT2.omega = 2*pi*LPF.PT2.fB; % [rad/s]
LPF.PT2.d = 0.5; % dampf factor
LPF.PT2.den = [1/LPF.PT2.omega^2 2*LPF.PT2.d/LPF.PT2.omega 1];
LPF.PT1.den = [1/LPF.PT2.omega 1];

% open('computeIMUNoiseRN');
sim('computeIMUNoiseRN');

% noise_acc_out
% noise_angRate_out
%% flash LiDAR noise
%从mat文件加载fL1MeasArrayRange
rng(1)
fL1MeasRangeNoiseStd = 0.05;
fL1MeasRangenoise = fL1MeasRangeNoiseStd*randn([256,256,(length(fL1MeasArrayRange.signals.values)-1)/100+1]);
for i = 1:length(fL1MeasArrayRange.signals.values)
    if(mod(i,100)==1)
        fL1MeasArrayRange.signals.values(:,:,i) = fL1MeasArrayRange.signals.values(:,:,i) + fL1MeasRangenoise(:,:,(i-1)/100+1);
    end
end
fL1MeasRange = fL1MeasArrayRange.signals.values;
%% flag
flNewData = 