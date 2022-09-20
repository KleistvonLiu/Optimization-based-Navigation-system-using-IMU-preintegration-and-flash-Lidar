
%Test and comparison file of
%Astrone-STARv20r\src\matlab\AttitudeConversion and \MathQuaternions library
%
%This code explains kinematic meaning of outputs of each function in Astrone-STARv20r
%
%Date: 05.02.2021
%       06.12.2021 updated
%Author:    Bangshang LIU

roll_n = 20 / 180 * pi; % should be in radian and DO NOT change!
pitch_n = 48 / 180 * pi;
yaw_n = 20 / 180 * pi;
R_x_n = [1 0 0;
       0 cos(roll_n) sin(roll_n);
       0 -sin(roll_n) cos(roll_n)];
   
R_y_n = [cos(pitch_n) 0 -sin(pitch_n);
       0 1 0;
       sin(pitch_n) 0 cos(pitch_n)];
   
R_z_n = [cos(yaw_n) sin(yaw_n) 0;
       -sin(yaw_n) cos(yaw_n) 0;
       0 0 1];
R_noise = R_x_n * R_y_n * R_z_n; % reference rotation from navigation to body frame (Rotation sequence: Z-Y'-X'')


%% --- test rotX rotY and rotZ
% R_temp = rotX(roll_n);
% Rx_lib = [R_temp(1,1:3);R_temp(2,1:3);R_temp(3,1:3)];
% delta_Rx = Rx_lib-R_x_n
% 
% R_temp = rotY(pitch_n);
% Ry_lib = [R_temp(1,1:3);R_temp(2,1:3);R_temp(3,1:3)];
% delta_Ry = Ry_lib-R_y_n
% 
% R_temp = rotZ(yaw_n);
% Rz_lib = [R_temp(1,1:3);R_temp(2,1:3);R_temp(3,1:3)];
% delta_Rz = Rz_lib-R_z_n


%% --- test dcm_eulerAngles

% R_noise;
% 
% eulAng = [yaw_n pitch_n roll_n];
% R_temp = dcm_eulerAngles(eulAng, 321);
% R_lib = [R_temp(1,1:3);R_temp(2,1:3);R_temp(3,1:3)];
% 
% delta_R = R_lib-R_noise


%% --- test dcm_quaternion

% theta_v = [10 20 15]' / 180 * pi;
% theta_s = norm(theta_v);
% nv = theta_v/theta_s;
% v = nv * sin(0.5 * theta_s);
% s = cos(0.5 * theta_s);
% 
% q = [v;s];
% R = quat2rotmliub(q);
% 
% q_lib = quat(s, v');
% q_lib.vector;
% q_lib.scalar;
% R_temp = dcm_quaternion(q_lib);
% R_lib = [R_temp(1,1:3);R_temp(2,1:3);R_temp(3,1:3)];
% 
% delta_R = R_lib-R




%% --- test eulerAngles_dcm
% R_noise;
% eulAng = rotm2eulAngliub(R_noise); %(phi;theta;psi)
% 
% eulAng_temp = eulerAngles_dcm(mat(R_noise), 321); % (psi theta phi)
% eulAng_lib = flip(eulAng_temp'); %
% 
% delta_eulAng = eulAng_lib-eulAng


%% --- test rotm2quatliub

% q = rotm2quatliub(R_noise)';
% 
% q_temp = quaternion_dcm(mat(R_noise));
% q_lib = [q_temp.vector q_temp.scalar]';
% 
% delta_q = q_lib - q





%% --- test quaternion normalization, multiplication, inversion, transpose

% roll_b = 5 / 180 * pi; % should be in radian and DO NOT change!
% pitch_b = 12 / 180 * pi;
% yaw_b = 24 / 180 * pi;
% R_x_b = [1 0 0;
%        0 cos(roll_b) sin(roll_b);
%        0 -sin(roll_b) cos(roll_b)];
%    
% R_y_b = [cos(pitch_b) 0 -sin(pitch_b);
%        0 1 0;
%        sin(pitch_b) 0 cos(pitch_b)];
%    
% R_z_b = [cos(yaw_b) sin(yaw_b) 0;
%        -sin(yaw_b) cos(yaw_b) 0;
%        0 0 1];
% R_b = R_x_b * R_y_b * R_z_b;
% q_temp_b = quaternion_dcm(mat(R_b));
% 
% q_temp = quaternion_dcm(mat(R_noise))
% q_norm = norm(q_temp)
% q_inv = inv(q_temp)
% q_trans = transpose(q_temp)
% 
% % multiplication
% q_ab = mtimes(q_temp, q_temp_b);
% R_ab = R_b * R_noise;
% 
% % q_ab = mtimes(q_temp_b, q_temp)
% % R_ab = R_noise * R_b;
% 
% q_ab_R = quaternion_dcm(mat(R_ab));
% 
% delta_q = [q_ab.vector q_ab.scalar]' - [q_ab_R.vector q_ab_R.scalar]'
% 



%% --- test Euler angles to quaternion

% quat  = eulAng2quatliub([roll_n, pitch_n, yaw_n]);
% % quat  = eul2quaternion([0, -8/180*pi, 0]);
% 
% R_q = quat2rotmliub(quat);
% R_eul = eulAng2rotmliub([roll_n, pitch_n, yaw_n]);
% 
% R_q - R_noise
% R_eul - R_noise

%% --- test quaternion to Euler angles
%-> not good function
quat  = eulAng2quatliub([roll_n, pitch_n, yaw_n]');
eulAng = quat2eulAngliub(quat);

eulAng - [roll_n, pitch_n, yaw_n]'

%% --- test sequence rotation


% R_total = eul2rotm([roll_n, pitch_n, yaw_n]);
% 
% R_z  = eul2rotm([0, 0, yaw_n]);
% R_xy = eul2rotm([roll_n, pitch_n, 0]);
% 
% R_total - R_xy * R_z


%% some functions

function R = quat2rotmliub(q)
%Author: Bangshang LIU
%Date: 07.11.2019
%
%Description:
%   Converting quaternion to rotation matrix
%
%Input:     q quaternion in format [qx qy qz qw;], qw is the skalar value
%Output:    R rotation matrix (forward transformation from navigation to local frame)

% Normalize the quaternions
q = q / norm(q);
R = zeros(3);

R = [q(1)^2+q(4)^2-q(2)^2-q(3)^2 2*(q(1)*q(2)+q(3)*q(4)) 2*(q(1)*q(3)-q(2)*q(4));
    2*(q(1)*q(2)-q(3)*q(4)) q(2)^2+q(4)^2-q(1)^2-q(3)^2 2*(q(2)*q(3)+q(1)*q(4));
    2*(q(1)*q(3)+q(2)*q(4)) 2*(q(2)*q(3)-q(1)*q(4)) q(3)^2+q(4)^2-q(1)^2-q(2)^2];

end

function quat = rotm2quatliub(R)
%Modifying Matlab function rotm2quat()
%ROTM2QUAT Convert rotation matrix to quaternion
%   Q = ROTM2QUAT(R) converts a 3D rotation matrix, R, into the corresponding
%   unit quaternion representation, Q. The input, R, is an 3-by-3-by-N matrix
%   containing N orthonormal rotation matrices.
%   The output, Q, is an N-by-4 matrix containing N quaternions. Each
%   quaternion is of the form q = [x y z w], with a scalar number as
%   the last value. Each element of Q must be a real number.
%
%   If the input matrices are not orthonormal, the function will
%   return the quaternions that correspond to the orthonormal matrices
%   closest to the imprecise matrix inputs.
%
%
%   Example:
%      % Convert a rotation matrix to a quaternion
%      R = [0 0 1; 0 1 0; -1 0 0];
%      q = rotm2quat(R)
%
%   References:
%   [1] I.Y. Bar-Itzhack, "New method for extracting the quaternion from a 
%       rotation matrix," Journal of Guidance, Control, and Dynamics, 
%       vol. 23, no. 6, pp. 1085-1087, 2000
%
%   See also quat2rotm

%   Copyright 2014-2016 The MathWorks, Inc.

%Author: Bangshang LIU
%Date: 19.12.2019

%#codegen
R = R';
%so that quaternion describes also rotation from navigation frame to local frame
%robotics.internal.validation.validateRotationMatrix(R, 'rotm2quat', 'R');


% Pre-allocate output
quat = zeros(size(R,3), 4, 'like', R);

% Calculate all elements of symmetric K matrix
K11 = R(1,1,:) - R(2,2,:) - R(3,3,:);
K12 = R(1,2,:) + R(2,1,:);
K13 = R(1,3,:) + R(3,1,:);
K14 = R(3,2,:) - R(2,3,:);

K22 = R(2,2,:) - R(1,1,:) - R(3,3,:);
K23 = R(2,3,:) + R(3,2,:);
K24 = R(1,3,:) - R(3,1,:);

K33 = R(3,3,:) - R(1,1,:) - R(2,2,:);
K34 = R(2,1,:) - R(1,2,:);

K44 = R(1,1,:) + R(2,2,:) + R(3,3,:);

% Construct K matrix according to paper
K = [...
    K11,    K12,    K13,    K14;
    K12,    K22,    K23,    K24;
    K13,    K23,    K33,    K34;
    K14,    K24,    K34,    K44];

K = K ./ 3;

% For each input rotation matrix, calculate the corresponding eigenvalues
% and eigenvectors. The eigenvector corresponding to the largest eigenvalue
% is the unit quaternion representing the same rotation.
for i = 1:size(R,3)
    [eigVec,eigVal] = eig(K(:,:,i),'vector');
    [~,maxIdx] = max(real(eigVal));
    quat(i,:) =...
        real([eigVec(1,maxIdx) eigVec(2,maxIdx) eigVec(3,maxIdx) eigVec(4,maxIdx)]);
    % By convention, always keep scalar quaternion element positive. 
    % Note that this does not change the rotation that is represented
    % by the unit quaternion, since q and -q denote the same rotation.
    if quat(i,4) < 0
        quat(i,:) = -quat(i,:);
    end
end

end

function eulAng  = rotm2eulAngliub(rotm)
phi     = atan2(rotm(2,3),rotm(3,3));
theta   = -asin(rotm(1,3));
psi     = atan2(rotm(1,2),rotm(1,1));
  
eulAng = [phi;theta;psi];
end

function quat  = eulAng2quatliub(eulAng)
phi     = eulAng(1);
theta   = eulAng(2);
psi     = eulAng(3);

axis = [0 0 1]';
angle = psi;
quat_z = [axis * sin(0.5 * angle); cos(0.5 * angle)]; 

axis = [0 1 0]';
angle = theta;
quat_y = [axis * sin(0.5 * angle); cos(0.5 * angle)]; 

axis = [1 0 0]';
angle = phi;
quat_x = [axis * sin(0.5 * angle); cos(0.5 * angle)]; 

L_quat_z = [quat_z(4) -quat_z(3) -quat_z(2) quat_z(1);
            quat_z(3) quat_z(4) -quat_z(1) quat_z(2);
            -quat_z(2) quat_z(1) quat_z(4) quat_z(3);
            -quat_z(1) -quat_z(2) -quat_z(3) quat_z(4)];
quat_zy = L_quat_z * quat_y;

L_quat_zy = [quat_zy(4) -quat_zy(3) -quat_zy(2) quat_zy(1);
            quat_zy(3) quat_zy(4) -quat_zy(1) quat_zy(2);
            -quat_zy(2) quat_zy(1) quat_zy(4) quat_zy(3);
            -quat_zy(1) -quat_zy(2) -quat_zy(3) quat_zy(4)];
quat = L_quat_zy * quat_x;

end

function rotm = eulAng2rotmliub(eulAng)
rotm = eye(3);
phi = eulAng(1); % should be in radian and DO NOT change!
theta = eulAng(2);
psi = eulAng(3);
Rx = [1 0 0;
       0 cos(phi) sin(phi);
       0 -sin(phi) cos(phi)];
   
Ry = [cos(theta) 0 -sin(theta);
       0 1 0;
       sin(theta) 0 cos(theta)];
   
Rz = [cos(psi) sin(psi) 0;
       -sin(psi) cos(psi) 0;
       0 0 1];
rotm = Rx * Ry * Rz;
end