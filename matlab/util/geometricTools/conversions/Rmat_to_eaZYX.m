function eaZYX = Rmat_to_eaZYX(rmat)
% euler angles given in rads


% Euler ZYX 
% Rz = [c1 -s1 0; s1 c1 0; 0 0 1];
% Ry = [c2 0 s2; 0 1 0; -s2 0 c2];
% Rx = [1 0 0; 0 c3 -s3; 0 s3 c3];

% Rz*Ry*Rx
% [ c1*c2, c1*s2*s3 - c3*s1, s1*s3 + c1*c3*s2]
% [ c2*s1, c1*c3 + s1*s2*s3, c3*s1*s2 - c1*s3]
% [   -s2,            c2*s3,            c2*c3]



if (1-abs(rmat(3,1))) > eps % c2!=0
    eaZYX(1) = atan2(rmat(2,1),rmat(1,1));
    eaZYX(3) = atan2(rmat(3,2),rmat(3,3));
    if abs(eaZYX(1))<eps %s1!=0
        eaZYX(2) = atan2(-rmat(3,1),rmat(2,1)/sin(eaZYX(1)));
    else
        eaZYX(2) = atan2(-rmat(3,1),rmat(1,1)/cos(eaZYX(1)));
    end
else %c2 = 0
    eaZYX(3) = 0;
    eaZYX(1) = atan2(-rmat(1,2),rmat(2,2));
    eaZYX(2) = sign(-rmat(3,1))*pi/2;
end
    
    
end