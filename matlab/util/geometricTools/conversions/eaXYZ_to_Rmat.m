function [ Rmat ] = eaXYZ_to_Rmat( eaXYZ )
% convert euler angles XYZ into a rotation matrix
% euler angles XYZ given in rad

c1 = cos(eaXYZ(1));
c2 = cos(eaXYZ(2));
c3 = cos(eaXYZ(3));

s1 = sin(eaXYZ(1));
s2 = sin(eaXYZ(2));
s3 = sin(eaXYZ(3));

Rmat = [[            c1*c2,           -c2*s1,     s2];...
        [ c3*s1 + c1*s2*s3, c1*c3 - s1*s2*s3, -c2*s3];...
        [ s1*s3 - c1*c3*s2, c1*s3 + c3*s1*s2,  c2*c3]];
    
end

