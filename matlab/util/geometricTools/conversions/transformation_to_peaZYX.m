function  pose = transformation_to_peaZYX(transf)
    eaZYX = Rmat_to_eaZYX(transf(1:3,1:3));
    
    x = transf(1,4);
    y = transf(2,4);
    z = transf(3,4);
    
    pose = [x,y,z,eaZYX];
end