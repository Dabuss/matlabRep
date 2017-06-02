function [Tf,J] = getTorquesFromSpatialForceIiwa(config, tcpEulerZYX, f )
% computes torques from spatial force f ([N;N;N;Nm;Nm;Nm])
% f is expressed in the robot base reference frame

J = getGeometricJacobianIiwa(config,tcpEulerZYX);
Tf = J'*f;



end

