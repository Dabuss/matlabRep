function rmat = eaZYX_to_Rmat(eaZYX)
% convert euler angles XYZ into a rotation matrix
% euler angles XYZ given in rad
c1 = cos(eaZYX(1));
c2 = cos(eaZYX(2));
c3 = cos(eaZYX(3));

s1 = sin(eaZYX(1));
s2 = sin(eaZYX(2));
s3 = sin(eaZYX(3));

%     validateattributes(a,{'numeric'},{'size',[1,1]})
%     validateattributes(b,{'numeric'},{'size',[1,1]})
%     validateattributes(c,{'numeric'},{'size',[1,1]})
    
    rmat = [[c1*c2  ,  c1*s2*s3 - s1*c3  ,  c1*s2*c3 + s1*s3];...
            [s1*c2  ,  s1*s2*s3 + c1*c3  ,  s1*s2*c3 - c1*s3];...
            [ -s2   ,    c2*s3           ,    c2*c3         ]];
    
end