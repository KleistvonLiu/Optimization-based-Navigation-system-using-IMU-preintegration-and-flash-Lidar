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
Bas=0;
Bgs=0;
% ！！！！应该是要改成成员变量的，那就不用这么麻烦了
%g=0;

window_size = 10;
c = 50;% frame size = 50 imu data points
e = estimator(window_size);

%load data IMUmeas.mat
load('D:\GOG\DA\code\IMU_Data_10052022\IMUmeas.mat')
dt = 0.001;
acc = IMUmeas.acc_IB_B_ref.Data;
gyr = IMUmeas.angRate_IB_B_ref.Data;

for i = 1:15000%length(acc)
    e.testProcessIMU(dt,acc(i,:)',gyr(i,:)');
    e.testProcessPC(c);
    Rs(:,:,i) =e.Rs(:,:,2);
    Ps(:,i) =e.Ps(:,2);
    Vs(:,i) =e.Ps(:,2);
end



function [Ps,Rs,Vs]=processIMU(dt,linear_acceleration,angular_velocity,ini,Rs,Ps,Vs,Bas,Bgs)
    % if obj.initial true, set initial acc0 and gyro0, the flag comes from
    % processPC。
    
    
    % if !icontainer(obj.frame_count), instancise and initialize
    % IntegrationBase(acc0,gyro0,bas[frame],bws[frame])
    if ini==1
        i1 = IntegrationBase(linear_acceleration,angular_velocity,0,0);
    end
    
    i1.push_back(dt,linear_acceleration,angular_velocity)
    
    j=ini;
    un_acc_0 = Rs(j) * (acc_0 - Bas) - g;
    un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs;% !! Bgs(j)
    un_gyr = un_gyr * dt;
    Rs(j+1) =Rs(j)*quat2rotm(quaternion(1,un_gyr(1),un_gyr(2),un_gyr(3)));
    un_acc_1 = Rs(j+1) * (linear_acceleration - Bas) - g;% !! Bas(j)
    un_acc = 0.5 * (un_acc_0 + un_acc_1);
    Ps(j+1) =Ps(j)+ dt * Vs(j) + 0.5 * dt * dt * un_acc;
    Vs(j+1) =Ps(j)+ dt * un_acc;
    
%     acc_0 = linear_acceleration;?????
%     应该是用来初始化IntegrationBase，每来一次数据都更新但是只有framecount+1的时候才用到
%     gyr_0 = angular_velocity;
end