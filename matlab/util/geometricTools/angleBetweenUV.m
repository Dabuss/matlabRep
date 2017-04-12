function theta = angleBetweenUV(u,v)
% u      :      is the first 
% v      :      is the second vector
% theta  :      is the angle between the vectors 

nv = norm(v);
nu = norm(u);

if nu*nv ==0 % singular case
    theta = 0;
    return
end
theta = 2 * atan(norm(u*nv - nu*v) / norm(u * nv + nu * v));


end