a = pi/15;
b = pi/2;
c = pi/12.5;
nIt = 2;

rmat = eulerZYXToRotationMatrix(a,b,c)

rmat1 = rmat;

for i = 1:nIt
    [a,b,c] = rotationMatrixToEulerZYX(rmat1)
    
    rmat1 = eulerZYXToRotationMatrix(a,b,c);
end
rmat1
label = ['difference after ', num2str(nIt) , ' iterations'];
display(rmat - rmat1, label);