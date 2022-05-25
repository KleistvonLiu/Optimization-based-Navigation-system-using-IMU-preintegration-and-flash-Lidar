function angle = R2ypr(R)
    % sequence: Z Y X
    yaw = atan2(R(2,1),R(1,1));
    pitch = atan2(-R(1,3),R(1,1)*cos(yaw)+R(1,2)*sin(yaw));
    roll = atan2(R(1,3) * sin(yaw) - R(2,3) * cos(yaw), -R(1,2) * sin(yaw) + R(2,2) * cos(yaw));
    angle = 180*[yaw,pitch,roll]/pi;
end