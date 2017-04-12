function v = normalizeVector(u)
% u    :    is the input vector
% v    :    is the normalized vector
    if length(u)~=3
        display('input argument is not a 3D vector');
        return;
    end
    nU = norm(u);
    
    if nU == 0
        v = u;
    else
        v = u/nU;
    end
end