function setIiwaConfigurationOnVrep(remAPI, clientID, conf, jointHandles)


remAPI.simxPauseCommunication(clientID,1);
r = remAPI.simxSetJointPosition(clientID, jointHandles(1), conf(1), remAPI.simx_opmode_oneshot);
r = remAPI.simxSetJointPosition(clientID, jointHandles(2), conf(2), remAPI.simx_opmode_oneshot);
r = remAPI.simxSetJointPosition(clientID, jointHandles(3), conf(3), remAPI.simx_opmode_oneshot);
r = remAPI.simxSetJointPosition(clientID, jointHandles(4), conf(4), remAPI.simx_opmode_oneshot);
r = remAPI.simxSetJointPosition(clientID, jointHandles(5), conf(5), remAPI.simx_opmode_oneshot);
r = remAPI.simxSetJointPosition(clientID, jointHandles(6), conf(6), remAPI.simx_opmode_oneshot);
r = remAPI.simxSetJointPosition(clientID, jointHandles(7), conf(7), remAPI.simx_opmode_oneshot);
remAPI.simxPauseCommunication(clientID,0);
        
end