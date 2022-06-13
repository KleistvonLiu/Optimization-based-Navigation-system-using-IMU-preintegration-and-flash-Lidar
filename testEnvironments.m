clear; % as always
%% run paramters.m first to load constant paramters
parameters
%% Testing integrationBase class
%% part 1: we can put instances of classes simply in a variable like this
i1 = IntegrationBase(0,0,0,0);
i1.delta_p=[1;2;3];
i2 = IntegrationBase(0,0,0,0);
i2.delta_p
% pay attention to following code!
% icontainer1 = {};
% icontainer1(1) = i1;
icontainer = [i1,i2];
i1.delta_p
icontainer(1).delta_p
%%
Rs = [];
Ps = [];
Vs = [];
Bas= [];
Bgs= [];
% ！！！！应该是要改成成员变量的，那就不用这么麻烦了
%g=0;

flag = 2;

window_size = 10;
c = 50;% frame size = 50 imu data points
e = estimator(window_size);

%load data IMUmeas.mat
load('D:\GOG\DA\code\IMU_Data_10052022\IMUmeas.mat')
acc = IMUmeas.acc_IB_B_ref.Data;
gyr = IMUmeas.angRate_IB_B_ref.Data;

datastep = 100;% downsampled data from 1000hz to 10hz
dt = 0.001*datastep;

% add bias min(mean(acc/gyr))/10
accbias = 1e-7 + randn(size(acc))*1e-6;%0;
gyrobias = 0 + randn(size(gyr))*5e-6;%0;%假设gyro像vins一样被cali了

acc = acc + accbias;
gyr = gyr + gyrobias;

acc = downsample(acc,datastep);
gyr = downsample(gyr,datastep);

enddata = length(acc);%length(acc)
step = 100;%every 100 imu data, we have 1 pc data

% i=1 时 初始化preintegrationbase(1) and Ps(1),then frame cout = 2,for i =
% 2-101,we call processIMU(preintegration 2 and also Ps(2)) for frame count 2,
% for i = 101 we call processPC, then frame count = 3, after i = 1001,
% frame count = 10.
for i = 1:enddata
    
    e.processIMU(dt,acc(i,:)',gyr(i,:)');
    
    if mod(i,step) == 1
        e.testProcessPC(flag,c);
        
        index = min(window_size,floor(i/step)+1);
        Rs(:,:,floor(i/step)+1) =e.Rs(:,:,index);
        Ps(:,floor(i/step)+1) =e.Ps(:,index);
        Vs(:,floor(i/step)+1) =e.Ps(:,index);
        Bas(:,floor(i/step)+1) =e.Bas(:,index);
        Bgs(:,floor(i/step)+1) =e.Bgs(:,index);
    end
    
%     if i < datastep
%         Rs(:,:,i) =e.Rs(:,:,2);
%         Ps(:,i) =e.Ps(:,2);
%         Vs(:,i) =e.Ps(:,2);
%     end
       
end
% if (flag == 1)
%     Ps10 = Ps(:,1:100:end).'; % 1000hz to 10hz
% end
%% pure mid integration results
flag1 = 1;
e1 = estimator(window_size);
for i = 1:enddata
    
    e1.processIMU(dt,acc(i,:)',gyr(i,:)');
    
    if mod(i,step) == 1
        e1.testProcessPC(flag1,c);
        
        Rs1(:,:,floor(i/step)+1) =e1.Rs(:,:,2);
        Ps1(:,floor(i/step)+1) =e1.Ps(:,2);
        Vs1(:,floor(i/step)+1) =e1.Ps(:,2);
    end
    
%     if i < datastep
%         Rs(:,:,i) =e.Rs(:,:,2);
%         Ps(:,i) =e.Ps(:,2);
%         Vs(:,i) =e.Ps(:,2);
%     end
end
Angles1 = rotm2eul(Rs1);
Qs1 = rotm2quat(Rs1);
%% run ins model from doctor liu
open('D:\GOG\DA\code\IMU_Data_10052022\simpleINS.slx');
IMUmeas.acc_IB_B_ref = IMUmeas.acc_IB_B_ref + accbias;
IMUmeas.angRate_IB_B_ref = IMUmeas.angRate_IB_B_ref + gyrobias;
sim('D:\GOG\DA\code\IMU_Data_10052022\simpleINS.slx');
Pins = posi_IB_I_ref.Data;
Qins = quat_IB_ref.Data;
Vins = vel_IB_I_ref.Data;

IMU2PCstep = 100;
Pins = downsample(Pins,IMU2PCstep);
Qins = downsample(Qins,IMU2PCstep);
Vins = downsample(Vins,IMU2PCstep);
Angleins = downsample(ang_Euler_IB_ref.Data,IMU2PCstep);
Rinsend = quat2rotmatrixliub(Qins(end,:));
%% test icp
eul = [5 -2.5 3]/180*pi;
rotmZYX = eul2rotm(eul)
A = eye(4);
paramsICP.maxIterations = 100;
paramsICP.initialTransform = A;
paramsICP.metric = 'PointToPlane';
paramsICP.doExtrapolate = true;%true;
paramsICP.inlierRatio = 0.7;%0.3;%0.95
paramsICP.thresHold = 0.16;
paramsICP.tolerance = [0.00001, 0.00009];%[0.001, 0.001];%[0.00001, 0.00009];%[0.001, 0.009]
paramsICP.verbose = false;
paramsICP.useDegree = false;
paramsICP.arrayDim = 85;%sqrt(size(fixedPc,1));

fixedPC = randn(paramsICP.arrayDim^2,3);
movingPc = (rotmZYX*fixedPC')'+[0.08,-0.1,0.05];

fixedPcMN = reshape(fixedPC, paramsICP.arrayDim, paramsICP.arrayDim, 3);
movingPcMN = reshape(movingPc, paramsICP.arrayDim, paramsICP.arrayDim,3);
%tic
[transform,rmse] = pcicpFL_LJF_V2(movingPcMN, fixedPcMN, paramsICP);
%
dp_com = transform(1:3,4);%!!!dR_com'*dp_com=-[0.08,-0.1,0.05]，定点的坐标变大，相当于坐标系沿反向移动
dR_com = transform(1:3,1:3);% dR_com是rotmZYX的转置
% 
