function [taskHandles,taskNames,ret] = vrep_getTasksHandles(clientID,vrep)
    % retrieves handles corresponding to tasks (on VREP, these tasks have
    % to contain 'task') and sort them in alphabetical order. also return
    % their correponding VREP names in taskNames cell array.
    
    % retrieve all dummies handles and names in vrep scene
    [ret,dummiesHandles,int,floats,dummyNames]=vrep.simxGetObjectGroupData(clientID,vrep.sim_object_dummy_type,0,vrep.simx_opmode_blocking);
    
    % sort them by alphabetical order (may not be necessary)
    [sortedDummyNames, sortedIndex] = sort(dummyNames(:,1));
    sortedDummies = dummiesHandles(sortedIndex);
    
    % extract thos dummies who contain the characters 'task' within their
    % name
    taskNames = sortedDummyNames(not(cellfun('isempty', strfind(sortedDummyNames, 'task')))); % array of sorted strings containing the names of the tasks
    taskHandles = sortedDummies(not(cellfun('isempty', strfind(sortedDummyNames, 'task'))))'; % vector of tasks handles sorted in the same order as tasksNames


end