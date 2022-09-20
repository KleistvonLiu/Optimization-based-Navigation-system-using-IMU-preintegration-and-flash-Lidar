%% compare q to R between Matlab and Liu
q = [0.7071 0.7071 0 0];% angle:90 grad; axis:[1 0 0],we assume that this q
% means that, a coordinate system is rotated from {A} to {B}, compute the
% point PinA = [0 1 1] in B, P doesn't move. and PinB should be [0 1 -1]

P = [0 1 1]; 

qm = quaternion(q);
ql = [q(2:4) q(1)];

Rm = quat2rotm(qm)
Rl = quat2rotmliub(ql)
% results1: Rm and Rl are inverse for each other
% conclusion1: the position of scalar in quaternion doesn't influence the
% angle axis.
Pm = Rm*P'%[0 -1  1]
Pl = Rl*P'%[0  1 -1]
% conclusion2: MATLAB uses robotic rotation matrix, R means coordinate
% transformation from B to A; liuB uses areospace R, which means coordinate
% transformation from A to B.
%% compare the result of quaternion mulplication and corresponding rotation
clear
q1 = [0.7071 0.7071 0 0];
q2 = [0.7071 0 0.7071 0];

qm1 = quaternion(q1);
qm2 = quaternion(q2);
qm3 = qm1*qm2;
ql1 = [q1(2:4) q1(1)];
ql2 = [q2(2:4) q2(1)];
ql3 = quatMultiplication(ql1',ql2');

Rm1 = quat2rotm(qm1);
Rm2 = quat2rotm(qm2);
Rm12 = Rm1*Rm2;
Rm3 = quat2rotm(qm3);

Rl1 = quat2rotmliub(ql1);
Rl2 = quat2rotmliub(ql2);
Rl3 = quat2rotmliub(ql3);
Rl21 = Rl2*Rl1;
Rl12 = Rl1*Rl2;
% result:Rl21 = Rl3, Rm12 = Rm3, Rl3 = inverse(Rm3),Rl12 无意义
% 两个四元数相乘后，无论四元数的形式，他们表达的轴角是相同的，只是转成R时的表达不同。
%% compare the result of quaternion mulplication
clear
q1 = [1 3 6 2];
q2 = [2 8 1 2];

qm1 = quaternion(q1);
qm2 = quaternion(q2);
qm3 = qm1*qm2;
qm3 = normalize(qm3);
ql1 = [q1(2:4) q1(1)];
ql2 = [q2(2:4) q2(1)];
ql3 = quatMultiplication(ql1',ql2');

% Rm1 = quat2rotm(qm1);
% Rm2 = quat2rotm(qm2);
% Rm12 = Rm1*Rm2;
% Rm3 = quat2rotm(qm3);
% 
% Rl1 = quat2rotmliub(ql1);
% Rl2 = quat2rotmliub(ql2);
% Rl3 = quat2rotmliub(ql3);
% Rl21 = Rl2*Rl1;
% Rl12 = Rl1*Rl2;
% result:qm3 is the same with ql3
% 两个四元数相乘后，无论四元数的形式，他们表达的轴角是相同的，只是转成R时的表达不同。
% quatMultiplication会自动normalize四元数
%% compare the result of rotm2quat
q = [0.7071 0.7071 0 0];% angle:90 grad; axis:[1 0 0],we assume that this q
% means that, a coordinate system is rotated from {A} to {B}, compute the
% point PinA = [0 1 1] in B, P doesn't move. and PinB should be [0 1 -1]

P = [0 1 1]; 

qm = quaternion(q);
ql = [q(2:4) q(1)];

Rm = quat2rotm(qm);
Rl = quat2rotmliub(ql);

qm1 = rotm2quat(Rm);
ql1 = rotm2quatliub(Rl);
ql2 = rotm2quatliub(Rm);
% results: qm is quaternion class, qm1 is array. its values are same
%% test using R to replace quaternion multiplication
% 刘博士用了下面的代码，对于ICP 我们希望的结果是 Rba（robotic），同时Rba = Rla * Rbl 
q_pre = [0.7071 0 0 0.7071 ]; % qal
q_pre_inv= [-q_pre(1) -q_pre(2) -q_pre(3) q_pre(4)];
R_qpre_inv = [q_pre_inv(4) -q_pre_inv(3) q_pre_inv(2) q_pre_inv(1);
    q_pre_inv(3) q_pre_inv(4) -q_pre_inv(1) q_pre_inv(2);
    -q_pre_inv(2) q_pre_inv(1) q_pre_inv(4) q_pre_inv(3);
    -q_pre_inv(1) -q_pre_inv(2) -q_pre_inv(3) q_pre_inv(4)];

% - rotational part
q_now = [0 0.7071 0 0.7071 ]';
q_d = R_qpre_inv * q_now;
q_d = q_d/norm(q_d);
R_base_cur = quat2rotmliub(q_d);% rotation matrix

R_pre = quat2rotmliub(q_pre)'; %是a到l的坐标变换
R_now = quat2rotmliub(q_now)';
R_base_cur2 = R_pre'*R_now;
% results: R_base_cur 与 R_base_cur2 互为转置。