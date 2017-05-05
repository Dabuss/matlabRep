function [ output_args ] = vrep_setFullSystemConfiguration(vrep, clientID, conf, B_handle, joints_handles)
% set iiwa + KMR configuration on VREP.
% angles have to be given in radians and positions in meters
% x_b = conf(1) (m)
% y_b = conf(2) (m)
% theta_b = conf(3) (rad)
% q1 = conf(4) (rad)
% q2 = conf(5) (rad)
% q3 = conf(6) (rad)
% q4 = conf(7) (rad)
% q5 = conf(8) (rad)
% q6 = conf(9) (rad)
% q7 = conf(10) (rad)


z_0_B = 0.70307016; % height of the base of the arm. Not exact (roughly assessed from vrep), needs more accuracy



vrep.simxPauseCommunication(clientID,1);

% set KMR position
r = vrep.simxSetObjectPosition(clientID,B_handle,-1,[conf(1),conf(2),z_0_B],vrep.simx_opmode_oneshot); % set x_b and y_b
r = vrep.simxSetObjectOrientation(clientID,B_handle,-1,[0,0,conf(3)],vrep.simx_opmode_oneshot); % set theta_b

% set Iiwa configuration
r = vrep.simxSetJointPosition(clientID, joints_handles(1), conf(4),  vrep.simx_opmode_oneshot); % set q1
r = vrep.simxSetJointPosition(clientID, joints_handles(2), conf(5),  vrep.simx_opmode_oneshot); %     :
r = vrep.simxSetJointPosition(clientID, joints_handles(3), conf(6),  vrep.simx_opmode_oneshot); %     :
r = vrep.simxSetJointPosition(clientID, joints_handles(4), conf(7),  vrep.simx_opmode_oneshot); %     :
r = vrep.simxSetJointPosition(clientID, joints_handles(5), conf(8),  vrep.simx_opmode_oneshot); %     :
r = vrep.simxSetJointPosition(clientID, joints_handles(6), conf(9),  vrep.simx_opmode_oneshot); %     :
r = vrep.simxSetJointPosition(clientID, joints_handles(7), conf(10), vrep.simx_opmode_oneshot); % set q7

vrep.simxPauseCommunication(clientID,0);



end

