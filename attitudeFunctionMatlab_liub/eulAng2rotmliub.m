function rotm = eulAng2rotmliub(eulAng)
% rotation sequence: Z-Y'-X''
% Input: eulAng [phi; theta; psi]
% Output: rotm

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