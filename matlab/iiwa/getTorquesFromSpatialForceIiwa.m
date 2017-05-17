function Tf = getTorquesFromSpatialForceIiwa(config, tcpEulerZYX, f )
% computes torques from spatial force f ([N;N;N;Nm;Nm;Nm])

J = getGeometricJacobianIiwa(config,tcpEulerZYX);
Tf = J'*f;



end

