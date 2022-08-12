q = [0.0000;
   -0.8835;
    0.0000;
    0.4684];
eul = quat2eulAngliub(q);%[3.141592653589793;-0.974973398891921;3.141592653589793]

rotm1 = quat2rotmliub(q)
rotm2 = eulAng2rotmliub(eul)