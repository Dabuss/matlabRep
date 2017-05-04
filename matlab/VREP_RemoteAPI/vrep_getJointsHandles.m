function [ jointsHandles, jointNames ] = vrep_getJointsHandles(clientID,vrep)
% retrieves joint handles in vector jointsHandles and their corresponding
% names in cell array jointNames

%         [r,j1] = vrep.simxGetObjectHandle(clientID,'A1',vrep.simx_opmode_blocking);
%         [r,j2] = vrep.simxGetObjectHandle(clientID,'A2',vrep.simx_opmode_blocking);
%         [r,j3] = vrep.simxGetObjectHandle(clientID,'A3',vrep.simx_opmode_blocking);
%         [r,j4] = vrep.simxGetObjectHandle(clientID,'A4',vrep.simx_opmode_blocking);
%         [r,j5] = vrep.simxGetObjectHandle(clientID,'A5',vrep.simx_opmode_blocking);
%         [r,j6] = vrep.simxGetObjectHandle(clientID,'A6',vrep.simx_opmode_blocking);
%         [r,j7] = vrep.simxGetObjectHandle(clientID,'A7',vrep.simx_opmode_blocking);
        
%         jointsHandles = [j1,j2,j3,j4,j5,j6,j7];

    [ret,jointsHandles,int,floats,jointNames]=vrep.simxGetObjectGroupData(clientID,vrep.sim_object_joint_type,0,vrep.simx_opmode_blocking);
        
        

end

