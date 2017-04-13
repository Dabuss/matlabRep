function transf = poseEulerZYXToTransf(pose)
%     validateattributes(pose,{'numeric'},{'size',[1,6]})
    
    x = pose(1);
    y = pose(2);
    z = pose(3);
    a = pose(4);
    b = pose(5);
    c = pose(6);
    
    rmat = eulerZYXToRotationMatrix(a,b,c);
    
    transf = [[rmat,[x;y;z]];[0,0,0,1]];
    
end