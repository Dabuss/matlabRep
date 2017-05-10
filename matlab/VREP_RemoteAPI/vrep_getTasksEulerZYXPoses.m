function [tasks,ret] = vrep_getTasksEulerZYXPoses(clientID, vrep, tasks_handles)
    % retrieves EulerZYX tasks poses from their vrep handles and stores
    % them in the tasks cell array
    
    ntasks = length(tasks_handles);
    tasks = cell(1,ntasks);
    
    ret = 1;
    for i = 1:ntasks
        cpt = 0;
        [ret,pos]=vrep.simxGetObjectPosition(clientID,tasks_handles(i),-1,vrep.simx_opmode_streaming);
        while (ret > 0 || cpt > 1000)
            [ret,pos]=vrep.simxGetObjectPosition(clientID,tasks_handles(i),-1,vrep.simx_opmode_streaming);
            cpt = cpt+1;
            pause(0.001);
        end
        tasks{i}(1:3) = pos*1000; %pos in mm
        pause(0.001);
    end
    
    
    for i = 1:ntasks
        cpt = 0;
        [ret,eaXYZ]=vrep.simxGetObjectOrientation(clientID,tasks_handles(i),-1,vrep.simx_opmode_streaming);
        while (ret > 0 || cpt > 1000)
            [ret,eaXYZ]=vrep.simxGetObjectOrientation(clientID,tasks_handles(i),-1,vrep.simx_opmode_streaming);
            cpt = cpt+1;
            pause(0.001);
        end
        tasks{i}(4:6) = Rmat_to_eaZYX(eaXYZ_to_Rmat(eaXYZ));
        pause(0.001);
    end
    

end

