function [Q] = computeIKIiwaFast( tcpOffsetEuler, destPoseEuler, swivel )
%This function is designed for iiwa mounted with pneumatic touch flange!!!

H_7_tcp = peaZYX_to_transformation(tcpOffsetEuler);
H_0_tcp = peaZYX_to_transformation(destPoseEuler);
H_0_7 = H_0_tcp*inverseTransformation(H_7_tcp);

pose = transformation_to_peaZYX(H_0_7);
x = pose(1); y = pose(2); z = pose(3); a = pose(4); b = pose(5); c = pose(6);


Q(1,1) = Q_1_1(a,b,c,swivel,x,y,z);
Q(2,1) = Q(1,1);                          
Q(3,1) = Q(1,1) + pi;                     
Q(4,1) = Q(1,1) + pi;                     

Q(1,2) = Q_1_2(a,b,c,swivel,x,y,z);
Q(2,2) = Q(1,2);
Q(3,2) = -Q(1,2);
Q(4,2) = -Q(1,2);

Q(1,3) = Q_1_3(a,b,c,swivel,x,y,z);
Q(2,3) = Q(1,3) + pi;                        
Q(3,3) = Q(1,3);                             
Q(4,3) = Q(1,3) + pi; 

Q(1,4) = Q_1_4(a,b,c,swivel,x,y,z);
Q(2,4) = -Q(1,4);
Q(3,4) = -Q(1,4);
Q(4,4) = Q(1,4);

Q(5:8,1:4) = Q(1:4,1:4)
%TODO

Q(1,5) = Q_1_5(a,b,c,Q(1,1),Q(1,2),Q(1,3),Q(1,4));


Q(1,6) = Q_1_6(a,b,c,Q(1,1),Q(1,2),Q(1,3),Q(1,4));


Q(1,7) = Q_1_7(a,b,c,Q(1,1),Q(1,2),Q(1,3),Q(1,4));




end

