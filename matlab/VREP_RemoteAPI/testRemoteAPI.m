%checkCenterOfGravityInVrep

displayInVrep = true;

% generateIiwa;


if displayInVrep
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
    
    if (clientID>-1)
        disp('Connected to remote API server');
        
        
        
        % retrieve joint handles
        [joints_handles, joints_names] = vrep_getJointsHandles(clientID,vrep);
        
        % retrieve KMR dummy handle
        [ret,B_handle] = vrep.simxGetObjectHandle(clientID,'B',vrep.simx_opmode_blocking);
        
        % retrieve tasks handles
        [tasks_handles,tasks_names] = vrep_getTasksHandles(clientID,vrep);

        % retrieve distances handles
        [distances_handles,distances_names] = vrep_getDistancesHandles(clientID,vrep);
        
        setVrep
        
        %% Set Joint angles on vrep 
%         for i=1:10
%             conf = 360*(1/2 - rand(1,7));
%             setIiwaConfigurationOnVrep(vrep, clientID, conf, jointHandles);
%             pause(0.1);
%         end
%         
%       
jp1=0.0;
% vrep.simxGetJointPosition(clientID,jointHandles(1),jp1,vrep.simx_opmode_buffer)
[ret,jp1]=vrep.simxGetJointPosition(clientID,joints_handles(1),vrep.simx_opmode_streaming);
jp1
[ret,h]=vrep.simxGetDistanceHandle(clientID, 'Col_Distance_KMR_scene', vrep.simx_opmode_blocking)

tic
for i=1:100
%     [ret,jp1]=vrep.simxGetJointPosition(clientID,jointHandles(1),vrep.simx_opmode_streaming);
%     jp1*180/pi;
        [ret, d_KMR_env] = vrep.simxReadDistance(clientID, h, vrep.simx_opmode_streaming);
        d_KMR_env
        pause(0.2);
end
toc   
        vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    disp('Program ended');
end %if displayInVrep