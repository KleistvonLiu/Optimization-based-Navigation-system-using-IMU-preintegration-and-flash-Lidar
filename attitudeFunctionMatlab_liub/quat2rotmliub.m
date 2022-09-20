function rotm = quat2rotmliub(quat)
%Author: Bangshang LIU
%Date: 07.11.2019
%
%Description:
%   Converting quaternion to rotation matrix
%
% Input: eulAng = [phi; theta; psi]
% Output: quat = [qx; qy; qz; qw]
% rotation sequence: Z-Y'-X''


% Normalize the quaternions
quat = quat / norm(quat);
rotm = zeros(3);

rotm = [quat(1)^2+quat(4)^2-quat(2)^2-quat(3)^2 2*(quat(1)*quat(2)+quat(3)*quat(4)) 2*(quat(1)*quat(3)-quat(2)*quat(4));
    2*(quat(1)*quat(2)-quat(3)*quat(4)) quat(2)^2+quat(4)^2-quat(1)^2-quat(3)^2 2*(quat(2)*quat(3)+quat(1)*quat(4));
    2*(quat(1)*quat(3)+quat(2)*quat(4)) 2*(quat(2)*quat(3)-quat(1)*quat(4)) quat(3)^2+quat(4)^2-quat(1)^2-quat(2)^2];

end