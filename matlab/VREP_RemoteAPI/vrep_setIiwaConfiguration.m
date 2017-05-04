function vrep_setIiwaConfiguration(vrep, clientID, conf, joints_handles)
% conf has to be given in radians

vrep.simxPauseCommunication(clientID,1);
r = vrep.simxSetJointPosition(clientID, joints_handles(1), conf(1), vrep.simx_opmode_oneshot);
r = vrep.simxSetJointPosition(clientID, joints_handles(2), conf(2), vrep.simx_opmode_oneshot);
r = vrep.simxSetJointPosition(clientID, joints_handles(3), conf(3), vrep.simx_opmode_oneshot);
r = vrep.simxSetJointPosition(clientID, joints_handles(4), conf(4), vrep.simx_opmode_oneshot);
r = vrep.simxSetJointPosition(clientID, joints_handles(5), conf(5), vrep.simx_opmode_oneshot);
r = vrep.simxSetJointPosition(clientID, joints_handles(6), conf(6), vrep.simx_opmode_oneshot);
r = vrep.simxSetJointPosition(clientID, joints_handles(7), conf(7), vrep.simx_opmode_oneshot);
vrep.simxPauseCommunication(clientID,0);
        
end