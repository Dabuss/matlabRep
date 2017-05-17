function Ts = getFkIiwaFast(config,tcpPoseEuler)
% uses symbolic toolbox generated functions to compute iiwa FK function

% generated from getFkIiwa

q1 = config(1); q2 = config(2); q3 = config(3); q4 = config(4); q5 = config(5); q6 = config(6); q7 = config(7); 
xEE = tcpPoseEuler(1); yEE = tcpPoseEuler(2); zEE = tcpPoseEuler(3); aEE = tcpPoseEuler(4); bEE = tcpPoseEuler(5); cEE = tcpPoseEuler(6);

Ts = {getH01Iiwa(q1), getH02Iiwa(q1,q2), getH03Iiwa(q1,q2,q3), getH04Iiwa(q1,q2,q3,q4), getH05Iiwa(q1,q2,q3,q4,q5), getH06Iiwa(q1,q2,q3,q4,q5,q6), getH07Iiwa(q1,q2,q3,q4,q5,q6,q7), getH0TCPIiwa(aEE,bEE,cEE,q1,q2,q3,q4,q5,q6,q7,xEE,yEE,zEE)};


end

