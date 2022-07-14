addpath('D:\GOG\DA\code\attitudeFunctionMatlab_liub');
%%
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

% icp configuration
A = eye(4);
paramsICP.maxIterations = 20;
paramsICP.initialTransform = A;
paramsICP.metric = 'PointToPlane';
paramsICP.doExtrapolate = true;%true;
paramsICP.inlierRatio = 0.7;%0.3;%0.95
paramsICP.thresHold = 0.16;
paramsICP.tolerance = [0.001, 0.009];%[0.001, 0.001];%[0.00001, 0.00009];%
paramsICP.verbose = false;
paramsICP.useDegree = false;
paramsICP.arrayDim = 85;%sqrt(size(fixedPc,1));

e = estimator(window_size);
e2 = estimatorv2(window_size);

%load data IMUmeas.mat
load('D:\GOG\DA\code\IMU_Data_10052022\IMUmeas.mat')
acc = IMUmeas.acc_IB_B_ref.Data;
gyr = IMUmeas.angRate_IB_B_ref.Data;

datastep = 100;% downsampled data from 1000hz to 10hz
dt = 0.001*datastep;

% add bias min(mean(acc/gyr))/10
rng(1)
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
    
    %e.processIMU(dt,acc(i,:)',gyr(i,:)');
    e2.processIMU(dt,acc(i,:)',gyr(i,:)');
    
    if mod(i,step) == 1
        %e.testProcessPC(flag,c);
        %e.ProcessPC(pc,params);
        e2.testProcessPC(flag,c);
        
%         index = min(window_size,floor(i/step)+1);
%         Rs(:,:,floor(i/step)+1) =e.Rs(:,:,index);
%         Ps(:,floor(i/step)+1) =e.Ps(:,index);
%         Vs(:,floor(i/step)+1) =e.Ps(:,index);
%         Bas(:,floor(i/step)+1) =e.Bas(:,index);
%         Bgs(:,floor(i/step)+1) =e.Bgs(:,index);
        
        index = min(window_size,floor(i/step)+1);
        Rs(:,:,floor(i/step)+1) =e2.Rs(:,:,index);
        Ps(:,floor(i/step)+1) =e2.Ps(:,index);
        Vs(:,floor(i/step)+1) =e2.Ps(:,index);
        Bas(:,floor(i/step)+1) =e2.Bas(:,index);
        Bgs(:,floor(i/step)+1) =e2.Bgs(:,index);
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
rotmZYX = eul2rotm(eul);
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
movingPc = (rotmZYX*fixedPC')'+[0.08,-0.1,0.05];%点云的点是绝对固定不变的，这里加的R和T实际上是观测物的位姿变化

fixedPcMN = reshape(fixedPC, paramsICP.arrayDim, paramsICP.arrayDim, 3);
movingPcMN = reshape(movingPc, paramsICP.arrayDim, paramsICP.arrayDim,3);
%tic
[transform,rmse] = pcicpFL_LJF_V2(movingPcMN, fixedPcMN, paramsICP);
%
dp_com = transform(1:3,4);%这就是{m}在{f}中的p
%!!!-dR_com'*dp_com= [0.08,-0.1,0.05]，定点的坐标变大，相当于坐标系沿反向移动
dR_com = transform(1:3,1:3);%这就是将{m}中的坐标变换成{f}中的坐标
test1 = sum(vecnorm(movingPc*dR_com.'+ dp_com.'-fixedPC,2,2));
%dR_com是rotmZYX的转置
% ICP求得应该是点云坐标系的变换或者说moving坐标系在fixed坐标系的位姿。
%% 我现在想要把之前使用robotic 惯例的Rq的代码 转成使用刘博Rq的代码，所以需要测试两者的结果是否相同
clear
% run paramters.m first to load constant paramters
parameters

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

% icp configuration
A = eye(4);
paramsICP.maxIterations = 20;
paramsICP.initialTransform = A;
paramsICP.metric = 'PointToPlane';
paramsICP.doExtrapolate = true;%true;
paramsICP.inlierRatio = 0.7;%0.3;%0.95
paramsICP.thresHold = 0.16;
paramsICP.tolerance = [0.001, 0.009];%[0.001, 0.001];%[0.00001, 0.00009];%
paramsICP.verbose = false;
paramsICP.useDegree = false;
paramsICP.arrayDim = 85;%sqrt(size(fixedPc,1));

e = estimator(window_size);
e2 = estimatorv2(window_size);

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
for i = 1:100
    
    e.processIMU(dt,acc(i,:)',gyr(i,:)');
    e2.processIMU(dt,acc(i,:)',gyr(i,:)');
    
    if i==1
    e.testProcessPC(flag,c);
    e2.testProcessPC(flag,c);
    end
        %e.ProcessPC(pc,params);

%     if i < datastep
%         Rs(:,:,i) =e.Rs(:,:,2);
%         Ps(:,i) =e.Ps(:,2);
%         Vs(:,i) =e.Ps(:,2);
%     end
       
end

index=2;

Rs =e.Rs(:,:,index);
Ps =e.Ps(:,index);
Vs =e.Ps(:,index);
Bas =e.Bas(:,index);
Bgs =e.Bgs(:,index);

Rs2 =e2.Rs(:,:,index);
Ps2 =e2.Ps(:,index);
Vs2 =e2.Ps(:,index);
Bas2 =e2.Bas(:,index);
Bgs2 =e2.Bgs(:,index);
%% 现在需要考虑在小行星上a和w都收到之前状态影响
clear
%% 
% load parameters first
parameters

Rs = [];
Ps = [];
Vs = [];
Bas= [];
Bgs= [];
g_sum= [];

% ！！！！应该是要改成成员变量的，那就不用这么麻烦了
%g=0;

window_size = 2;
c = 50;% frame size = 50 imu data points

%load data 
load('D:\GOG\DA\code\usefuldatafromNavFramework.mat')

%%
% initial pose of B in L
X_init = [-0.010835766809501;-0.037451635197423;-7.547757199546605;0;0;0;0;0;-0.698100000000000;0.716000000000000];

% true imu measurements
acctrue = acc.accTrue.data;
acctrue = downsample(acctrue,100);
gyrotrue = gyro.angRateTrue.data;
gyrotrue = downsample(gyrotrue,100);

% imu measurements with noise,10hz
accnoised = accMeas.signals.values;%转换成simulink model的时候应该不用特别提取value，参考刘博的KF里面代码
gyrnoised = angRateMeas.signals.values;
dt = 0.1;

% select imu measurements
useTrueIMUdata = 0;
if (useTrueIMUdata == 1)
    accdata = acctrue;
    gyrodata = gyrotrue;
else
    accdata = accnoised;
    gyrodata = gyrnoised;
end

% gravity
gravity_T = gravity_input.data;
gravity_T = downsample(gravity_T,100);


% flash LiDAR measurements
fL1MeasArrayDir = fL1Meas_dir;
fL1MeasRange = fL1MeasArrayRange.signals.values;
fL1MeasFlagNewData = 1;% 可以把这一步分单独放到if里面，然后flprocess和processPC放到if外面
navMode = 1;%不为96即可
flagUseTrueRelStates = 1;%
fLArrayDim = 256;
computeICP2base = 0;

%sepearte icp configuration, because 2last is easier than 2base
A = eye(4);
params4base.maxIterations = 20;
params4base.initialTransform = A;
params4base.metric = 'PointToPlane';
params4base.doExtrapolate = true;%true;
params4base.inlierRatio = 0.7;%0.3;%0.95
params4base.thresHold = 0.16;
params4base.tolerance = [0.00001, 0.00009];%[0.001, 0.009];%[0.00001, 0.00009];%
params4base.verbose = false;
params4base.useDegree = false;
params4base.arrayDim = 85;%sqrt(size(fixedPc,1));

params4last.maxIterations = 20;
params4last.initialTransform = A;
params4last.metric = 'PointToPlane';
params4last.doExtrapolate = true;%true;
params4last.inlierRatio = 0.7;%0.3;%0.95
params4last.thresHold = 0.16;
params4last.tolerance = [0.00001, 0.00009];%[0.001, 0.009];%[0.00001, 0.00009];%
params4last.verbose = false;
params4last.useDegree = false;
params4last.arrayDim = 85;%sqrt(size(fixedPc,1));

enddata = 5001; %length(accdata);%length(acc)
step = 100;%every 100 imu data, we have 1 pc data
%%
% instance of estimator
% e2 = estimatorv2(window_size, X_init);
%% 
% %转换成simulink model的时候可以把for 里面的东西直接复制，参考刘博KF修改，主要是保存上一帧的状态last_x
% 
% % i=1 时 初始化preintegrationbase(1) and Ps(1),then frame cout = 2,for i =
% % 2-101,we call processIMU(preintegration 2 and also Ps(2)) for frame count 2,
% % for i = 101 we call processPC, then frame count = 3, after i = 1001,
% % frame count = 10.
% flag = 2;
% 
% for i = 1:enddata
%     
%     %e.processIMU(dt,acc(i,:)',gyr(i,:)');
%     e2.processIMU(dt,accdata(i,:)',gyrodata(i,:)',gravity_T(i,:)');
%     
%     if mod(i,step) == 1
%         
% %         pose_ref = zeros(6,1);
% %         pose_ref(1:3) = posi_LB_L_ref(:,i);
% %         pose_ref(4:6) = eulAng_LB_ref(:,i);
% %         Xn_pose_cur = 
% %         
% %         [deltaX_icp2base, deltaX_icp2last,flagRegFinished, consec2base,...
% %             deltaX2base_ref, deltaX2last_ref,numValidPoints, mHRF] = fLProcessing(...
% %             pose_ref, Xn_pose_cur,...
% %             fL1MeasArrayDir, fL1MeasArrayRange, fL1MeasFlagNewData,...
% %             navMode, flagUseTrueRelStates,...                               %input
% %             fLArrayDim, fL1Pose_B, X_init,params4base,params4last);
%         
%         %e.testProcessPC(flag,c);
%         %e.ProcessPC(pc,params);
%         e2.testProcessPC(flag,c);
%         
%         %在高于window_size + 1的时候，由于ProcessPC先滑窗，此时Rs实际上应该是第十帧的值，
%         %但是第十一帧的初值也是第十帧的end，所以没事，但是对于g_sum。每一帧的初值都是0，所以应该是window_size
%          index = min(window_size,floor(i/step)+1);
% %         Rs(:,:,floor(i/step)+1) =e2.Rs(:,:,index);
% %         Ps(:,floor(i/step)+1) =e2.Ps(:,index);
% %         Vs(:,floor(i/step)+1) =e2.Ps(:,index);
% %         Bas(:,floor(i/step)+1) =e2.Bas(:,index);
% %         Bgs(:,floor(i/step)+1) =e2.Bgs(:,index);
%          g_sum(:,floor(i/step)+1) =e2.g_sum(:,index);
%     end
%             
%     index = min(window_size + 1,ceil(i/step)+1);
%     Rs(:,:,i) =e2.Rs(:,:,index);
%     Ps(:,i) =e2.Ps(:,index);
%     Vs(:,i) =e2.Vs(:,index);
%     Bas(:,i) =e2.Bas(:,index);
%     Bgs(:,i) =e2.Bgs(:,index);
%     
%     
% 
% %     if i < datastep
% %         Rs(:,:,i) =e.Rs(:,:,2);
% %         Ps(:,i) =e.Ps(:,2);
% %         Vs(:,i) =e.Ps(:,2);
% %     end
%        
% end
% % if (flag == 1)
% %     Ps10 = Ps(:,1:100:end).'; % 1000hz to 10hz
% % end
%% 相较于estimatorv2将所有q的保存格式从行变为了列

% instance of estimator
e3 = estimatorv3(window_size, X_init);


clear fLProcessing % clear persistent variables

flag = 3;%1 只有一帧，测试中值积分功能;2只有11帧，测试积分以及滑窗以及只有IMU的优化功能。3可变滑窗以及PC优化功能

for i = 1:enddata
    
    %e.processIMU(dt,acc(i,:)',gyr(i,:)');
    e3.processIMU(dt,accdata(i,:)',gyrodata(i,:)',gravity_T(i,:)');
    
    if mod(i,step) == 1
        
        pose_ref = zeros(6,1);
        pose_ref(1:3) = posi_LB_L_ref(:,i);
        pose_ref(4:6) = eulAng_LB_ref(:,i);
        [P,~,R,~,~,~] = e3.outputState();
        Xn_pose_cur = [P; rotm2quatliub(R.')];
        
        [deltaX_icp2base, deltaX_icp2last,flagRegFinished, consec2base,...
            deltaX2base_ref, deltaX2last_ref,numValidPoints, mHRF] = fLProcessing(...
            pose_ref, Xn_pose_cur,...
            fL1MeasArrayDir, fL1MeasRange(:,:,i), fL1MeasFlagNewData,...
            navMode, flagUseTrueRelStates,computeICP2base,...                               %input
            fLArrayDim, fL1Pose_B, X_init,params4base,params4last);
        
        % g_sum(:,floor(i/step)+1) =e2.outputDeltaG();%需要放在processPc前，因为新的帧的g_sum是0
        
        %e.testProcessPC(flag,c);
        %e.ProcessPC(pc,params);
        e3.testProcessPC(flag,c,deltaX_icp2base, deltaX_icp2last);
        
         
    end
            
    %index = min(window_size + 1,ceil(i/step)+1);
    [P,Q,R,V,Ba,Bg] = e3.outputState();
    Rs(:,:,i) = R;
    Ps(:,i) = P;
    Vs(:,i) = V;
    Bas(:,i) = Ba;
    Bgs(:,i) = Bg;
    
    
       
end
% if (flag == 1)
%     Ps10 = Ps(:,1:100:end).'; % 1000hz to 10hz
% end

% 求最大欧氏距离差
er1 = max(vecnorm(Ps(:,1:enddata))-vecnorm(posi_LB_L_ref(:,1:enddata)));
er2 = max(vecnorm(posi_LB_L_est(:,1:enddata))-vecnorm(posi_LB_L_ref(:,1:enddata)));
% 求最终欧氏距离差
% er1 = vecnorm(Ps(:,enddata))-vecnorm(posi_LB_L_ref(:,enddata));
% er2 = vecnorm(posi_LB_L_est(:,enddata))-vecnorm(posi_LB_L_ref(:,enddata));

%er2 = sum(abs(posi_LB_L_est(:,1:enddata)-posi_LB_L_ref(:,1:enddata)),'all');
%% pure mid integration results
e3 = estimatorv3(window_size, X_init);

flag1 = 1;
% e1 = estimator(window_size);
for i = 1:enddata
    
%     if i ==5000
%         keyboard 
%     end
    
    e3.processIMU(dt,accdata(i,:)',gyrodata(i,:)',gravity_T(i,:)');
    
    %if mod(i,step) == 1
    if i == 1    
        e3.testProcessPC(flag1,c);
%         Rs1(:,:,floor(i/step)+1) =e2.Rs(:,:,2);
%         Ps1(:,floor(i/step)+1) =e2.Ps(:,2);
%         Vs1(:,floor(i/step)+1) =e2.Ps(:,2);
    end
    Rs1(:,:,i) =e3.Rs(:,:,2);
    Ps1(:,i) =e3.Ps(:,2);
    Vs1(:,i) =e3.Ps(:,2);

end
% Angles1 = rotm2eul(Rs1);
Qs1 = rotm2quatliub(Rs1);

% 求最大欧氏距离差
er3 = max(vecnorm(Ps1(:,1:enddata))-vecnorm(posi_LB_L_ref(:,1:enddata)));
% 求最终欧氏距离差
% er3 = vecnorm(Ps1(:,enddata))-vecnorm(posi_LB_L_ref(:,enddata));

%er3 = sum(abs(Ps1(:,1:enddata)-posi_LB_L_ref(:,1:enddata)),'all');
%% （过去的代码）estimator 增加了outputState 和outputdeltag的功能，测试下能否正常运行
%转换成simulink model的时候可以把for 里面的东西直接复制，参考刘博KF修改，主要是保存上一帧的状态last_x

% i=1 时 初始化preintegrationbase(1) and Ps(1),then frame cout = 2,for i =
% 2-101,we call processIMU(preintegration 2 and also Ps(2)) for frame count 2,
% for i = 101 we call processPC, then frame count = 3, after i = 1001,
% frame count = 10.

clear fLProcessing % clear persistent variables

flag = 3;%1 只有一帧，测试中值积分功能;2只有11帧，测试积分以及滑窗以及只有IMU的优化功能。3可变滑窗以及PC优化功能

for i = 1:enddata
    
    %e.processIMU(dt,acc(i,:)',gyr(i,:)');
    e2.processIMU(dt,accdata(i,:)',gyrodata(i,:)',gravity_T(i,:)');
    
    if mod(i,step) == 1
        
        pose_ref = zeros(6,1);
        pose_ref(1:3) = posi_LB_L_ref(:,i);
        pose_ref(4:6) = eulAng_LB_ref(:,i);
        [P,~,R,~,~,~] = e2.outputState();
        Xn_pose_cur = [P; rotm2quatliub(R.')];
        
        [deltaX_icp2base, deltaX_icp2last,flagRegFinished, consec2base,...
            deltaX2base_ref, deltaX2last_ref,numValidPoints, mHRF] = fLProcessing(...
            pose_ref, Xn_pose_cur,...
            fL1MeasArrayDir, fL1MeasRange(:,:,i), fL1MeasFlagNewData,...
            navMode, flagUseTrueRelStates,computeICP2base,...                               %input
            fLArrayDim, fL1Pose_B, X_init,params4base,params4last);
        
        % g_sum(:,floor(i/step)+1) =e2.outputDeltaG();%需要放在processPc前，因为新的帧的g_sum是0
        
        %e.testProcessPC(flag,c);
        %e.ProcessPC(pc,params);
        e2.testProcessPC(flag,c,deltaX_icp2base, deltaX_icp2last);
        
        %在高于window_size + 1的时候，由于ProcessPC先滑窗，此时Rs实际上应该是第十帧的值，
        %但是第十一帧的初值也是第十帧的end，所以没事，但是对于g_sum。每一帧的初值都是0，所以应该是window_size
         %index = min(window_size,floor(i/step)+1);
%         Rs(:,:,floor(i/step)+1) =e2.Rs(:,:,index);
%         Ps(:,floor(i/step)+1) =e2.Ps(:,index);
%         Vs(:,floor(i/step)+1) =e2.Ps(:,index);
%         Bas(:,floor(i/step)+1) =e2.Bas(:,index);
%         Bgs(:,floor(i/step)+1) =e2.Bgs(:,index);
         
    end
            
    %index = min(window_size + 1,ceil(i/step)+1);
    [P,Q,R,V,Ba,Bg] = e2.outputState();
    Rs(:,:,i) = R;
    Ps(:,i) = P;
    Vs(:,i) = V;
    Ba(:,i) = Ba;
    Bg(:,i) = Bg;
    
    

%     if i < datastep
%         Rs(:,:,i) =e.Rs(:,:,2);
%         Ps(:,i) =e.Ps(:,2);
%         Vs(:,i) =e.Ps(:,2);
%     end
       
end
% if (flag == 1)
%     Ps10 = Ps(:,1:100:end).'; % 1000hz to 10hz
% end