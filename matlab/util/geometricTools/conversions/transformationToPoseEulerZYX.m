function [x,y,z,a,b,c] = transformationToPoseEulerZYX(transf)
    [a,b,c] = rotationMatrixToEulerZYX(transf(1:3,1:3));
    x = transf(1,4);
    y = transf(2,4);
    z = transf(3,4);
end