a = 0.1;b=0.2;c=0.12;
rmat=eulerZYXToRotationMatrix(a,b,c)
[a1,b1,c1] = rotationMatrixToEulerZYX(rmat)