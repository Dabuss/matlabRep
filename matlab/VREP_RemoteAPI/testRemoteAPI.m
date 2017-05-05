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
        
        
        conf = [(1/2-rand(1,2))*3,(1/2-rand())*2*pi,(1/2-rand(1,7))*2*pi];
        % set KMR config
        vrep_setKMRConfiguration(vrep, clientID, conf(1:3), B_handle);
        
        % set iiwa config
        vrep_setIiwaConfiguration(vrep, clientID, conf(4:10), joints_handles);
        
        % set full system config
        vrep_setFullSystemConfiguration(vrep, clientID, -conf, B_handle, joints_handles);
        
        % get tasks EulerZYX poses
        tasks = vrep_getTasksEulerZYXPoses(clientID, vrep, tasks_handles);
        
        % get distances from collision and autocollision
        distances_values = vrep_getDistancesToObstacles(clientID, vrep, distances_handles);
        
        
        vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    disp('Program ended');
end %if displayInVrep