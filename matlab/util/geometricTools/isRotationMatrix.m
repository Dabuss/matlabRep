function bool = isRotationMatrix(mat)
    % tests wether mat is a rotation matrix
    epsilon = 10^-5;
    
    validateattributes(mat,{'numeric'},{'size',[3,3]})
%     bool = isequal(mat, transpose(mat));
    bool = norm(cross(mat(1:3,1),mat(1:3,2))-mat(1:3,3)) < epsilon;
    
    if bool
        bool = (abs ( norm(mat(1:3,1)) - 1 )< epsilon);
    end
end