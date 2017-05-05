function [ eaXYZ ] = Rmat_to_eaXYZ( R )
% euler angles given in rads

% [            c1*c2,           -c2*s1,     s2]
% [ c3*s1 + c1*s2*s3, c1*c3 - s1*s2*s3, -c2*s3]
% [ s1*s3 - c1*c3*s2, c1*s3 + c3*s1*s2,  c2*c3]

% epsilon=1e-12;

if (1-abs(R(1,3))>eps) % <=> abs(c2)>epsilon
    eaXYZ(1) = atan2(-R(1,2),R(1,1));
    eaXYZ(3) = atan2(-R(2,3),R(3,3));
    if abs(eaXYZ(1))>eps % <=> abs(s1)>epsilon
        eaXYZ(2) = atan2(R(1,3),-R(1,2)/sin(eaXYZ(1)));
    else % <=> abs(s1)<epsilon <=> theta_1 = 0
        eaXYZ(2) = atan2(R(1,3),R(1,1)/cos(eaXYZ(1)));
    end
else % abs(s2) = 1 <=> theta_2 = pi/2 ou -pi/2
    eaXYZ(3) = 0; % on choisit arbitrairement
    eaXYZ(1) = atan2(R(2,1),R(2,2));
    if R(1,3)>0
        eaXYZ(2) = pi/2;
    else
        eaXYZ(2) = -pi/2;
    end
end
    
endll


