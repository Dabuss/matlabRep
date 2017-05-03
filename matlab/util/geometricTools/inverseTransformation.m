function invTransf = inverseTransformation(transf)
% transf     :      the transformation to be inversed
% invTransf  :      the inverse of transf
%     validateattributes(transf,{'numeric'},{'size',[4,4]})
    
    
    rT = transpose(transf(1:3,1:3));
    t = transf(1:3,4);
    
    
    invTransf = [[rT,-rT*t];[0 0 0 1]];
end