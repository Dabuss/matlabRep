function eaZYX = Rmat_to_eaZYX(rmat)
%     if (~isRotationMatrix(rmat))
%         a = NaN; b = NaN; c = Nan;
%         return;
%     end
    
    
     
    sy = sqrt(rmat(1,1) * rmat(1,1) +  rmat(2,1) * rmat(2,1)); % tests if r11 and r21 are both equal to 0
     
    singular = sy < 1e-6;
 
    if  ~singular
%         a = atan2(rmat(3,2) , rmat(3,3));
%         b = atan2(-rmat(3,1), sy);
%         c = atan2(rmat(2,1), rmat(1,1));
          eaZYX(2) = atan2(-rmat(3,1),sy);
          cb = cos(eaZYX(2));
          eaZYX(1) = atan2(rmat(2,1)/cb,rmat(1,1)/cb);
          eaZYX(3) = atan2(rmat(3,2)/cb, rmat(3,3)/cb);
    else 
        eaZYX(1) = atan2(-rmat(2,3), rmat(2,2));
        eaZYX(2) = atan2(-rmat(3,1), sy);
        eaZYX(3) = 0;
    end
    
    
end