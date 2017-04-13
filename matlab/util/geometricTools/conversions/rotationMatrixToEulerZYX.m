function [a, b, c] = rotationMatrixToEulerZYX(rmat)
    if (~isRotationMatrix(rmat))
        a = NaN; b = NaN; c = Nan;
        return;
    end
    
    
     
    sy = sqrt(rmat(1,1) * rmat(1,1) +  rmat(2,1) * rmat(2,1)); % tests if r11 and r21 are both equal to 0
     
    singular = sy < 1e-6;
 
    if  ~singular
%         a = atan2(rmat(3,2) , rmat(3,3));
%         b = atan2(-rmat(3,1), sy);
%         c = atan2(rmat(2,1), rmat(1,1));
          b = atan2(-rmat(3,1),sy);
          a = atan2(rmat(2,1)/cos(b),rmat(1,1)/cos(b));
          c = atan2(rmat(3,2)/cos(b), rmat(3,3)/cos(b));
    else 
        a = atan2(-rmat(2,3), rmat(2,2));
        b = atan2(-rmat(3,1), sy);
        c = 0;
    end
    
    
end