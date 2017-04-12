function v = vectorProjectedOnPlane(u,n)
% u       :     is the vector to be projected
% n       :     is a normal vector of the plane
% v       :     is the normalized orthogonal projection of u along n

    
    v = normalizeVector(cross(cross(n,u),n));
    % /!\ Beware, if u is aligned to u, the projected vector is null

    
    
    
%     if (abs(v(1))<epsilon && abs(v(2))<epsilon && abs(v(3))<epsilon) % if n is colinear to u ...
%         v = normalizeVector(cross(n,[1,0,0]));
%         
%         if (abs(v(1))<epsilon && abs(v(2))<epsilon && abs(v(3))<epsilon)
%             v = normalizeVector(cross(n,[0,1,0]));
%         end
%         
%     end
    
    
end