function p_out = quatMultiplication(p, q)
%Author: Bangshang LIU
%Date: 07.05.2021
%
%Description:
%   Quaternion multiplication
%
%Input:     p quaternion in format [px py pz pw]', pw is the skalar value
%           q quaternion in format [qx qy qz qw]', qw is the skalar value
%Output:    p_out resulted quaternion in format [x y z w]', w is the skalar value

% Normalize the quaternions
p = p / norm(p);
q = q / norm(q);

L_p = [p(4)     -p(3)	p(2)	p(1);
       p(3)     p(4)	-p(1)	p(2);
       -p(2)    p(1)	p(4)	p(3);
       -p(1)	-p(2)	-p(3)	p(4)];
p_out = L_p * q;

end