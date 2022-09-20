function eulAng = quat2eulAngliub(quat)
%Modifying Matlab function quat2eul()
%According to book "The Global Positioning System and Inertial Navigation"
%from Jay Farrell, Matthew Barth, chapter 2 and "NASA Euler Angles, Quaternions, and Transformation Matrices 1977"
%
%Author: Bangshang LIU
%Date: 04.11.2019
%Version:   newly built             04.11.2019
%           bug fixed               05.01.2022
%
%Description:
%   Converting quaternion to Euler angles
%
%Input:     q quaternion in format [qx qy qz qw;], qw is the skalar value
%Output:    eul Euler angles in radians in format [phi theta psi;]. Sequence of Euler
%               angles are Z-Y'-X'' (psi-theta-phi) from old to new frame
%               (forward transformation)

% Normalize the quaternions
% q = robotics.internal.normalizeRows(q);
quat = quat/norm(quat);

qx = quat(1);
qy = quat(2);
qz = quat(3);
qw = quat(4);

% Pre-allocate output
eulAng = zeros(3, 1, 'like', quat);

% Cap all inputs to asin to 1, since values >1 produce complex
% results
% Since the quaternion is of unit length, this should never happen,
% but some code generation configuration seem to hit this edge case
% under some circumstances.
aSinInput = -2*(qx.*qz-qw.*qy);
aSinInput(aSinInput > 1) = 1;
aSinInput(aSinInput < -1) = -1;
        
eulAng = [ atan2( 2*(qy.*qz+qw.*qx), qw.^2 - qx.^2 - qy.^2 + qz.^2 ),...
            asin( aSinInput ), ...
            atan2( 2*(qx.*qy+qw.*qz), qw.^2 + qx.^2 - qy.^2 - qz.^2 )]';

% Check for complex numbers
if ~isreal(eulAng)
    eulAng = real(eulAng);
end

end