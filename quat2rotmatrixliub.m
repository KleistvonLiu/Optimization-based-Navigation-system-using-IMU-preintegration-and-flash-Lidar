function R = quat2rotmatrixliub(q)
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