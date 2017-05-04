function vrep_setKMRConfiguration(vrep, clientID, conf, B_handle)
% sets the configuration of the base, given the dummy handle B_handle
% x_b = conf(1) (mm)
% y_b = conf(2) (mm)
% theta_b = conf(3) (rad)


% [r,p_0_B]=vrep.simxGetObjectPosition(clientID,B_handle,-1,vrep.simx_opmode_blocking); % could be hardcoded
% z_0_B = p_0_B(3); % (m)

z_0_B = 0.70307016;

vrep.simxPauseCommunication(clientID,1);
r = vrep.simxSetObjectPosition(clientID,B_handle,-1,[conf(1),conf(2),z_0_B],vrep.simx_opmode_oneshot);
r = vrep.simxSetObjectOrientation(clientID,B_handle,-1,[0,0,conf(3)],vrep.simx_opmode_oneshot);
vrep.simxPauseCommunication(clientID,0);
        
end