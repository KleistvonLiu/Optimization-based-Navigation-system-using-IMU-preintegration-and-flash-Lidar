function eulAng  = rotm2eulAngliub(rotm)
% rotation sequence: Z-Y'-X''
% Input: rotm
% Output: eulAng [phi; theta; psi]

phi     = atan2(rotm(2,3),rotm(3,3));
theta   = -asin(rotm(1,3));
psi     = atan2(rotm(1,2),rotm(1,1));
  
eulAng = [phi;theta;psi];
end