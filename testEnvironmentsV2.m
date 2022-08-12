addpath('D:\GOG\DA\code\attitudeFunctionMatlab_liub');
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
angles = [];
NodeChange = [];
P_LB_L = [];
R_LB = [];

% ！！！！应该是要改成成员变量的，那就不用这么麻烦了
%g=0;

window_size = 5;
c = 50;% frame size = 50 imu data points

%load data 
%load('D:\GOG\DA\code\data\usefuldatafromNavFramework.mat')
load('D:\GOG\DA\code\data\simResults02_FS04_Case20useful.mat')


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
flagUseTrueRelStates = 0;%
fLArrayDim = 256;
computeICP2base = 1;

%sepearte icp configuration, because 2last is easier than 2base
A = eye(4);
params4base.maxIterations = 50;
params4base.initialTransform = A;
params4base.metric = 'PointToPlane';
params4base.doExtrapolate = false;%true;
params4base.inlierRatio = 0.7;%0.3;%0.95
params4base.thresHold = 0.16;
params4base.tolerance = [0.00001, 0.00009];%[0.001, 0.009];%[0.00001, 0.00009];%
params4base.verbose = false;
params4base.useDegree = false;
params4base.arrayDim = 85;%sqrt(size(fixedPc,1));

params4last.maxIterations = 20;
params4last.initialTransform = A;
params4last.metric = 'PointToPlane';
params4last.doExtrapolate = false;%true;
params4last.inlierRatio = 0.7;%0.3;%0.95
params4last.thresHold = 0.16;
params4last.tolerance = [0.00001, 0.00009];%[0.001, 0.009];%[0.00001, 0.00009];%
params4last.verbose = false;
params4last.useDegree = false;
params4last.arrayDim = 85;%sqrt(size(fixedPc,1));

enddata = 15001;%length(accdata); %length(accdata);%length(acc)
step = 100;%every 100 imu data, we have 1 pc data
erricp2base = zeros(enddata,1);
erricp2last = zeros(enddata,1);
for i = 1:size(quat_LB_est,2)
    
    eulAng_LB_est(:,i) = rad2deg(quat2eulAngliub(quat_LB_est(:,i)));
    
end
for i = 1:length(eulAng_LB_ref)
    R_LB_ref(:,:,i) = eulAng2rotmliub(eulAng_LB_ref(:,i));
end
% for i = 1:size(eulAng_LB_ref,2)
%     
%     eulAng_LB_ref(:,i) = rad2deg(eulAng_LB_ref(:,i));
%     
% end
acc_LB_L_ref = downsample(acc_LB_L_ref,100);
acc_LB_L_ref = acc_LB_L_ref.';
%% 相较于estimatorv3,v4是基于relative navigation的

e4 = estimatorv4(window_size, X_init);


clear fLProcessing % clear persistent variables

flag = 3;%1 只有一帧，测试中值积分功能;2只有11帧，测试积分以及滑窗以及只有IMU的优化功能。3可变滑窗以及PC优化功能

for i = 1:enddata
    
    %e.processIMU(dt,acc(i,:)',gyr(i,:)');
    e4.processIMU(dt,accdata(i,:)',gyrodata(i,:)',gravity_T(i,:)');
    flagNewNode = 0;
    if mod(i,step) == 1
%         if i == 1601
%            keyboard 
%         end
        pose_ref = zeros(6,1);
        pose_ref(1:3) = posi_LB_L_ref(:,i);
        pose_ref(4:6) = eulAng_LB_ref(:,i);
        [P,~,R,~,~,~] = e4.outputState();
        Xn_pose_cur = [P; rotm2quatliub(R.')];
        
        [deltaX_icp2base(:,floor(i/100)+1), deltaX_icp2last(:,floor(i/100)+1),flagRegFinished, consec2base,...
            deltaX2base_ref(:,floor(i/100)+1), deltaX2last_ref(:,floor(i/100)+1),flagNewNode,erricp2base(i),erricp2last(i),...
            numValidPoints, mHRF,deltaX_icp2base_g(:,floor(i/100)+1),deltaX_icp2last_g(:,floor(i/100)+1)] ...
            = fLProcessing(...
            pose_ref, Xn_pose_cur,...
            fL1MeasArrayDir, fL1MeasRange(:,:,i), fL1MeasFlagNewData,...
            navMode, flagUseTrueRelStates,computeICP2base,...                               %input
            fLArrayDim, fL1Pose_B, X_init,params4base,params4last);
        
        % g_sum(:,floor(i/step)+1) =e2.outputDeltaG();%需要放在processPc前，因为新的帧的g_sum是0
        
        %e.testProcessPC(flag,c);
        %e.ProcessPC(pc,params);
        e4.testProcessPC(flag,c,deltaX_icp2base(:,floor(i/100)+1), deltaX_icp2last(:,floor(i/100)+1),flagNewNode);
        
        if(flagNewNode==1)
            [P,~,R,~,~,~] = e4.outputState();
            Xn_pose_cur = [P; rotm2quatliub(R.')];
            X_initNew = e4.outputXNewInit();
            [~] = fLProcessing(...
                pose_ref, Xn_pose_cur,...
                fL1MeasArrayDir, fL1MeasRange(:,:,i), fL1MeasFlagNewData,...
                navMode, flagUseTrueRelStates,computeICP2base,...                               %input
                fLArrayDim, fL1Pose_B, X_initNew,params4base,params4last);
        end
         
    end
            
    %index = min(window_size + 1,ceil(i/step)+1);
    [P,Q,R,V,Ba,Bg] = e4.outputState();
    Rs(:,:,i) = R;
    Ps(:,i) = P;
    Vs(:,i) = V;
    Bas(:,i) = Ba;
    Bgs(:,i) = Bg;
    angles_N(:,i) = rad2deg(rotm2eulAngliub(permute(R,[2,1,3])));    
    NodeChange(i) = flagNewNode;   
    P_LB_L(:,i) = e4.posi_LB_L;
    R_LB(1:3,1:3,i) = e4.R_LB;
    angles_L(:,i) = rad2deg(rotm2eulAngliub(e4.R_LB)); 
end

% 求最大的差欧氏距离
er1 = max(vecnorm(P_LB_L(:,1:enddata)-posi_LB_L_ref(:,1:enddata)));
er2 = max(vecnorm(posi_LB_L_est(:,1:enddata)-posi_LB_L_ref(:,1:enddata)));

% 求最终欧氏距离差
% er5 = vecnorm(Ps(:,enddata)-posi_LB_L_ref(:,enddata));
% er6 = vecnorm(posi_LB_L_est(:,enddata)-posi_LB_L_ref(:,enddata));
% 求欧氏距离差的平均值
er3 = sum(vecnorm(P_LB_L(:,1:enddata)-posi_LB_L_ref(:,1:enddata)))/enddata;
er4 = sum(vecnorm(posi_LB_L_est(:,1:enddata)-posi_LB_L_ref(:,1:enddata)))/enddata;

tSim = linspace(0,floor(enddata/10),enddata); 

%% compute reference pose in N

posi_NB_N_ref = posi_LB_L_ref;
vel_NB_N_ref = vel_LB_L_ref;
R_NB_ref = R_LB_ref;
angles_ref_N = rad2deg(eulAng_LB_ref);
index = find(NodeChange);
R_LN = eye(3);
%effGrav_T = gravity_T - cross(angRate_IT_T,cross(angRate_IT_T, posi_TL_T + R_LT* obj.posi_LB_L));

for i = 1:length(index)
    nodeindex = index(i);
    effGrav_T = gravity_T(nodeindex,:)' - cross(angRate_IT_T,cross(angRate_IT_T, posi_TL_T + R_LT*posi_LB_L_ref(:,nodeindex)));
    effGrav_L = R_LT' * effGrav_T;
    effGrav_N = R_LN * effGrav_L;
    %求Nj2Ni
    dir_N1 = [0 0 1]';
    dir_Grav = effGrav_N/norm(effGrav_N);
    rotAng = acos(dot(dir_N1, dir_Grav));
    rotaxis_temp = cross(dir_N1,dir_Grav);
    rotaxis = rotaxis_temp / norm(rotaxis_temp);
    quat_N2_1Grav = [rotaxis * sin(rotAng/2); cos(rotAng/2)];
    R_N1N2 = quat2rotmliub(quat_N2_1Grav);% N1到N2的坐标变换
    posi_N1N2_N1 = posi_NB_N_ref(:,nodeindex);
    for j = nodeindex:enddata
       posi_NB_N_ref(:,j) =  R_N1N2*(posi_NB_N_ref(:,j) - posi_N1N2_N1);
       vel_NB_N_ref(:,j) = R_N1N2*vel_NB_N_ref(:,j);
       R_NB_ref(:,:,j) = R_NB_ref(:,:,j)*R_N1N2';
       angles_ref_N(:,j) = rad2deg(rotm2eulAngliub(R_NB_ref(:,:,j))); 
    end
    R_LN = R_N1N2*R_LN;
end
ern1 = sum(vecnorm(Ps(:,1:enddata)-posi_NB_N_ref(:,1:enddata)))/enddata;

h(98) = figure('Name','Position in N');
subplot(3,2,1)
tSim = linspace(0,floor(enddata/10),enddata); 
plot(tSim, Ps(1,1:enddata),'black'); hold on;grid on;
plot(tSim, posi_NB_N_ref(1,1:enddata),':r'); hold on;grid on;
title('Position Estimation in N');
ylabel('x [m]','FontSize',50);set(gca,'FontSize',15);
legend('optEst','Ref. in N frame')
subplot(3,2,3)
plot(tSim, Ps(2,1:enddata),'black'); hold on;grid on;
plot(tSim, posi_NB_N_ref(2,1:enddata),':r'); hold on;grid on;
legend('optEst','Ref. in N frame')
ylabel('y [m]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,5)
plot(tSim, Ps(3,1:enddata),'black'); hold on;grid on;
plot(tSim, posi_NB_N_ref(3,1:enddata),':r'); hold on;grid on;
legend('optEst','Ref. in N frame')
ylabel('z [m]','FontSize',50);set(gca,'FontSize',15);
xlabel('time [s]','FontSize',20);

subplot(3,2,2)
plot(tSim, Ps(1,1:enddata)-posi_NB_N_ref(1,1:enddata)); hold on;grid on;
title('Position errors in N');
ylabel('x (est-ref) [m]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,4)
plot(tSim, Ps(2,1:enddata)-posi_NB_N_ref(2,1:enddata)); hold on;grid on;
ylabel('y (est-ref) [m]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,6)
plot(tSim, Ps(3,1:enddata)-posi_NB_N_ref(3,1:enddata)); hold on;grid on;
ylabel('z (est-ref) [m]','FontSize',50);set(gca,'FontSize',15);
xlabel('time [s]','FontSize',20);

h(97) = figure('Name','Norm of position difference(est-ref) in N');
% subplot(3,2,1)
% tSim = linspace(0,500,enddata); 
plot(tSim, vecnorm(Ps(:,1:enddata)-posi_NB_N_ref(:,1:enddata)),'b'); hold on;grid on;
title('Norm of position difference(est-ref) in N');
ylabel('x [m]','FontSize',50);set(gca,'FontSize',15);
legend('optEst')

h(96) = figure('Name','Euler angles in N');
subplot(3,2,1)
plot(tSim, angles_N(1,1:enddata),'black'); hold on;grid on;
plot(tSim, angles_ref_N(1,1:enddata),':r'); hold on;grid on;
title('Euler angles Estimation in N');
ylabel('roll [degree]','FontSize',50);set(gca,'FontSize',15);
legend('optEst','Ref. in N frame')
subplot(3,2,3)
plot(tSim, angles_N(2,1:enddata),'black'); hold on;grid on;
plot(tSim, angles_ref_N(2,1:enddata),':r'); hold on;grid on;
legend('optEst','Ref. in N frame')
ylabel('pitch [degree]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,5)
plot(tSim, angles_N(3,1:enddata),'black'); hold on;grid on;
plot(tSim, angles_ref_N(3,1:enddata),':r'); hold on;grid on;
legend('optEst','Ref. in N frame')
ylabel('yaw [degree]','FontSize',50);set(gca,'FontSize',15);
xlabel('time [s]','FontSize',20);

subplot(3,2,2)
plot(tSim, angles_N(1,1:enddata)-angles_ref_N(1,1:enddata)); hold on;grid on;
title('Euler angles errors in N');
ylabel('x (est-ref) [degree]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,4)
plot(tSim, angles_N(2,1:enddata)-angles_ref_N(2,1:enddata)); hold on;grid on;
ylabel('y (est-ref) [degree]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,6)
plot(tSim, angles_N(3,1:enddata)-angles_ref_N(3,1:enddata)); hold on;grid on;
ylabel('z (est-ref) [degree]','FontSize',50);set(gca,'FontSize',15);
xlabel('time [s]','FontSize',20);
%% visulization
h(1) = figure('Name','Position');
subplot(3,2,1)
tSim = linspace(0,floor(enddata/10),enddata); 
plot(tSim, posi_LB_L_est(1,1:enddata),'b'); hold on;grid on;
plot(tSim, P_LB_L(1,1:enddata),'black'); hold on;grid on;
plot(tSim, posi_LB_L_ref(1,1:enddata),':r'); hold on;grid on;
title('Position Estimation');
ylabel('x [m]','FontSize',50);set(gca,'FontSize',15);
legend('fLaINSest','optEst','Ref. in L frame')
subplot(3,2,3)
plot(tSim, posi_LB_L_est(2,1:enddata),'b'); hold on;grid on;
plot(tSim, P_LB_L(2,1:enddata),'black'); hold on;grid on;
plot(tSim, posi_LB_L_ref(2,1:enddata),':r'); hold on;grid on;
legend('fLaINSest','optEst','Ref. in L frame')
ylabel('y [m]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,5)
plot(tSim, posi_LB_L_est(3,1:enddata),'b'); hold on;grid on;
plot(tSim, P_LB_L(3,1:enddata),'black'); hold on;grid on;
plot(tSim, posi_LB_L_ref(3,1:enddata),':r'); hold on;grid on;
legend('fLaINSest','optEst','Ref. in L frame')
ylabel('z [m]','FontSize',50);set(gca,'FontSize',15);
xlabel('time [s]','FontSize',20);

subplot(3,2,2)
plot(tSim, P_LB_L(1,1:enddata)-posi_LB_L_ref(1,1:enddata)); hold on;grid on;
title('Position errors');
ylabel('x (est-ref) [m]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,4)
plot(tSim, P_LB_L(2,1:enddata)-posi_LB_L_ref(2,1:enddata)); hold on;grid on;
ylabel('y (est-ref) [m]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,6)
plot(tSim, P_LB_L(3,1:enddata)-posi_LB_L_ref(3,1:enddata)); hold on;grid on;
ylabel('z (est-ref) [m]','FontSize',50);set(gca,'FontSize',15);
xlabel('time [s]','FontSize',20);

h(2) = figure('Name','Norm of position difference(est-ref)');
% subplot(3,2,1)
% tSim = linspace(0,500,enddata); 
plot(tSim, vecnorm(posi_LB_L_est(:,1:enddata)-posi_LB_L_ref(:,1:enddata)),'r'); hold on;grid on;
plot(tSim, vecnorm(P_LB_L(:,1:enddata)-posi_LB_L_ref(:,1:enddata)),'b'); hold on;grid on;
title('Norm of position difference(est-ref)');
ylabel('x [m]','FontSize',50);set(gca,'FontSize',15);
legend('fLaINSest','optEst')

h(3) = figure('Name','Velocity');
subplot(3,2,1)
%tSim = linspace(0,500,enddata); 
plot(tSim, vel_LB_L_est(1,1:enddata),'b'); hold on;grid on;
plot(tSim, Vs(1,1:enddata),'black'); hold on;grid on;
plot(tSim, vel_LB_L_ref(1,1:enddata),':r'); hold on;grid on;
title('Velocity Estimation');
ylabel('x [m/s]','FontSize',50);set(gca,'FontSize',15);
legend('fLaINSest','optEst','Ref. in L frame')
subplot(3,2,3)
plot(tSim, vel_LB_L_est(2,1:enddata),'b'); hold on;grid on;
plot(tSim, Vs(2,1:enddata),'black'); hold on;grid on;
plot(tSim, vel_LB_L_ref(2,1:enddata),':r'); hold on;grid on;
legend('fLaINSest','optEst','Ref. in L frame')
ylabel('y [m/s]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,5)
plot(tSim, vel_LB_L_est(3,1:enddata),'b'); hold on;grid on;
plot(tSim, Vs(3,1:enddata),'black'); hold on;grid on;
plot(tSim, vel_LB_L_ref(3,1:enddata),':r'); hold on;grid on;
legend('fLaINSest','optEst','Ref. in L frame')
ylabel('z [m/s]','FontSize',50);set(gca,'FontSize',15);
xlabel('time [s]','FontSize',20);

subplot(3,2,2)
plot(tSim, Vs(1,1:enddata)-vel_LB_L_ref(1,1:enddata)); hold on;grid on;
title('Velocity errors');
ylabel('x (est-ref) [m/s]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,4)
plot(tSim, Vs(2,1:enddata)-vel_LB_L_ref(2,1:enddata)); hold on;grid on;
ylabel('y (est-ref) [m/s]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,6)
plot(tSim, Vs(3,1:enddata)-vel_LB_L_ref(3,1:enddata)); hold on;grid on;
ylabel('z (est-ref) [m/s]','FontSize',50);set(gca,'FontSize',15);
xlabel('time [s]','FontSize',20);

h(4) = figure('Name','Euler angles');
subplot(3,2,1)
plot(tSim, eulAng_LB_est(1,1:enddata),'b'); hold on;grid on;
plot(tSim, angles_L(1,1:enddata),'black'); hold on;grid on;
plot(tSim, rad2deg(eulAng_LB_ref(1,1:enddata)),':r'); hold on;grid on;
title('Euler angles Estimation');
ylabel('roll [degree]','FontSize',50);set(gca,'FontSize',15);
legend('fLaINSest','optEst','Ref. in L frame')
subplot(3,2,3)
plot(tSim, eulAng_LB_est(2,1:enddata),'b'); hold on;grid on;
plot(tSim, angles_L(2,1:enddata),'black'); hold on;grid on;
plot(tSim, rad2deg(eulAng_LB_ref(2,1:enddata)),':r'); hold on;grid on;
legend('fLaINSest','optEst','Ref. in L frame')
ylabel('pitch [degree]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,5)
plot(tSim, eulAng_LB_est(3,1:enddata),'b'); hold on;grid on;
plot(tSim, angles_L(3,1:enddata),'black'); hold on;grid on;
plot(tSim, rad2deg(eulAng_LB_ref(3,1:enddata)),':r'); hold on;grid on;
legend('fLaINSest','optEst','Ref. in L frame')
ylabel('yaw [degree]','FontSize',50);set(gca,'FontSize',15);
xlabel('time [s]','FontSize',20);

subplot(3,2,2)
plot(tSim, angles_L(1,1:enddata)-rad2deg(eulAng_LB_ref(1,1:enddata))); hold on;grid on;
title('Euler angles errors');
ylabel('x (est-ref) [degree]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,4)
plot(tSim, angles_L(2,1:enddata)-rad2deg(eulAng_LB_ref(2,1:enddata))); hold on;grid on;
ylabel('y (est-ref) [degree]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,6)
plot(tSim, angles_L(3,1:enddata)-rad2deg(eulAng_LB_ref(3,1:enddata))); hold on;grid on;
ylabel('z (est-ref) [degree]','FontSize',50);set(gca,'FontSize',15);
xlabel('time [s]','FontSize',20);

h(5) = figure('Name','Ref. acceleration in L frame');
subplot(3,1,1)
plot(tSim, acc_LB_L_ref(1,1:enddata),'b'); hold on;grid on;
% plot(tSim, refRN_Vel_BL(1,1:numStepSim),':r'); hold on;grid on;
title('Acc. in L-frame');
ylabel('x [m/s^2]');
% legend('Navigation','Reference')
subplot(3,1,2)
plot(tSim, acc_LB_L_ref(2,1:enddata),'b'); hold on;grid on;
% plot(tSim, refRN_Vel_BL(2,1:numStepSim),':r'); hold on;grid on;
ylabel('y [m/s^2]');
% legend('Navigation','Reference')
subplot(3,1,3)
plot(tSim, acc_LB_L_ref(3,1:enddata),'b'); hold on;grid on;
% plot(tSim, refRN_Vel_BL(3,1:numStepSim),':r'); hold on;grid on;
ylabel('z [m/s^2]');
% legend('Navigation','Reference')
xlabel('time [s]');

h(6) = figure('Name','Angular rates LB in B');
subplot(3,1,1)
% plot(tSim, rad2deg(angRate_LB_B_est(1,1:enddata)),'b'); hold on;grid on;
plot(tSim, rad2deg(angRate_LB_B_ref(1,1:enddata)),'b'); hold on;grid on;
title('INS angular rates LB in B');
ylabel('phi [deg/s]');
% legend('fLaINSest','Ref.')
legend('Ref.')
subplot(3,1,2)
% plot(tSim, rad2deg(angRate_LB_B_est(2,1:enddata)),'b'); hold on;grid on;
plot(tSim, rad2deg(angRate_LB_B_ref(2,1:enddata)),'b'); hold on;grid on;
ylabel('theta [deg/s]');
% legend('fLaINSest','Ref.')
legend('Ref.')
subplot(3,1,3)
% plot(tSim, rad2deg(angRate_LB_B_est(3,1:enddata)),'b'); hold on;grid on;
plot(tSim, rad2deg(angRate_LB_B_ref(3,1:enddata)),'b'); hold on;grid on;
ylabel('psi [deg/s]');
% legend('fLaINSest','Ref.')
legend('Ref.')
xlabel('time [s]');

h(7) = figure('Name','Node change');
plot(tSim,NodeChange(1:enddata));

h(99) = figure('Name','icp error');
subplot(2,1,1);
plot(tSim,erricp2base(1:enddata));
legend('ICP to base PC')

subplot(2,1,2);
plot(tSim,erricp2last(1:enddata));
legend('ICP to last PC')

%saveas(gcf,'simResults03_FS04_Case20useful.fig');
%% pure mid integration results
e4 = estimatorv3(window_size, X_init);

flag1 = 1;
% e1 = estimator(window_size);
for i = 1:enddata
    
%     if i ==5000
%         keyboard 
%     end
    
    e4.processIMU(dt,accdata(i,:)',gyrodata(i,:)',gravity_T(i,:)');
    
    %if mod(i,step) == 1
    if i == 1    
        e4.testProcessPC(flag1,c);
%         Rs1(:,:,floor(i/step)+1) =e2.Rs(:,:,2);
%         Ps1(:,floor(i/step)+1) =e2.Ps(:,2);
%         Vs1(:,floor(i/step)+1) =e2.Ps(:,2);
    end
    Rs1(:,:,i) =e4.Rs(:,:,2);
    Ps1(:,i) =e4.Ps(:,2);
    Vs1(:,i) =e4.Ps(:,2);

end
% Angles1 = rotm2eul(Rs1);
Qs1 = rotm2quatliub(Rs1);

% 求最大欧氏距离差
er7 = max(abs(vecnorm(Ps1(:,1:enddata)-posi_LB_L_ref(:,1:enddata))));
% 求最终欧氏距离差
% er7 = vecnorm(Ps1(:,enddata))-vecnorm(posi_LB_L_ref(:,enddata));

%er7 = sum(abs(Ps1(:,1:enddata)-posi_LB_L_ref(:,1:enddata)),'all');