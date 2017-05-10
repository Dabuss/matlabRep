function [ eaXYZ ] = Rmat_to_eaXYZ( R )
% euler angles given in rads


% Euler XYZ 
% Rx = [1 0 0; 0 c1 -s1; 0 s1 c1];
% Ry = [c2 0 s2; 0 1 0; -s2 0 c2];
% Rz = [c3 -s3 0; s3 c3 0; 0 0 1];

% Rx*Ry*Rz
% [            c2*c3,           -c2*s3,     s2]
% [ c1*s3 + c3*s1*s2, c1*c3 - s1*s2*s3, -c2*s1]
% [ s1*s3 - c1*c3*s2, c3*s1 + c1*s2*s3,  c1*c2]



if (1-abs(R(1,3))>eps) % <=> abs(c2)>epsilon
    eaXYZ(3) = atan2(-R(1,2),R(1,1));
    eaXYZ(1) = atan2(-R(2,3),R(3,3));
    if abs(eaXYZ(1))>eps % <=> abs(s1)>epsilon
        eaXYZ(2) = atan2(R(1,3),-R(2,3)/sin(eaXYZ(1)));
    else % <=> abs(s1)<epsilon <=> theta_1 = 0
        eaXYZ(2) = atan2(R(1,3),R(3,3)/cos(eaXYZ(1)));
    end
else % abs(s2) = 1 <=> theta_2 = pi/2 ou -pi/2
    eaXYZ(3) = 0; % on choisit arbitrairement
    eaXYZ(1) = atan2(R(3,2),R(2,2));
    eaXYZ(2) = sign(R(1,3))*pi/2;
end
    

end
