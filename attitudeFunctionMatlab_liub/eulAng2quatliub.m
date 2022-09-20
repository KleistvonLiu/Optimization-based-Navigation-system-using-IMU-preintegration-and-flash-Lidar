function quat  = eulAng2quatliub(eulAng)
% rotation sequence: Z-Y'-X''
% Input: eulAng = [phi; theta; psi]
% Output: quat = [qx; qy; qz; qw]

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