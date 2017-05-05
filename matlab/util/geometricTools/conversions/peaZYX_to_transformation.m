function transf = peaZYX_to_transformation(pose)
%     validateattributes(pose,{'numeric'},{'size',[1,6]})
 
    
    rmat = eaZYX_to_Rmat([pose(4),pose(5),pose(6)]);
    
    transf = [[rmat,pose(1:3)'];[0,0,0,1]];
    
end