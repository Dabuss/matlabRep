classdef vrepStore_miiwa
    
    properties
        %vrep specific
        vrep;
        clientID;
        
        %joint handles
        joints_handles;
        joints_names;
        B_handle;
        
        %distance object handles
        distances_handles;
        distances_names;
        
        %tasks positions, handles and names
        tasks;
        tasks_handles
        tasks_names;
        
        
    end
    
    
    methods
        %---------------------
        % contructor
        %---------------------
        function obj = vrepStore_miiwa()
            obj.vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            obj.vrep.simxFinish(-1); % just in case, close all opened connections
            obj.clientID=obj.vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
            
            if (obj.clientID>-1)
                disp(' ')
                disp('Connected to remote API server ...');
                disp(' ');
                
                % retrieve joint handles
                [obj.joints_handles, obj.joints_names] = vrep_getJointsHandles(obj.clientID,obj.vrep);
                disp('    -iiwa joint handles retrieved');
                
                
                % retrieve KMR dummy handle
                [ret,obj.B_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'B',obj.vrep.simx_opmode_blocking);
                disp('    -KMR Base handles retrieved');
                
                
                % retrieve distances handles
                [obj.distances_handles,obj.distances_names] = vrep_getDistancesHandles(obj.clientID,obj.vrep);
                disp('    -distances handles retrieved');
                
                
                % get tasks EulerZYX poses
                obj.tasks = vrep_getTasksEulerZYXPoses(obj.clientID, obj.vrep, obj.tasks_handles);
                disp('    -tasks handles retrieved');
                
                
                % retrieve tasks handles
                [obj.tasks_handles,obj.tasks_names] = vrep_getTasksHandles(obj.clientID,obj.vrep);
                disp('    -tasks poses retrieved');
                
                disp(' ');
                disp('OK, you may start using me now...');
                
            else
                obj.vrep.delete(); % call the destructor!
                disp('Program ended');
            
            end % error on clientID retrieval
        
        end % contructor
        
        
        
        
        
        
        %---------------------
        % destructor
        %---------------------
        function delete(obj)
            disp('already bored of me ?');
            
            obj.vrep.simxFinish(obj.clientID);
            obj.vrep.delete(); 
            
            disp('object vrepStore_miiwa destroyed');
        end
        
        
        
        
        
        %---------------------
        % testFunctionality
        %---------------------
        function testFunctionality(obj)
            conf = [(1/2-rand(1,2))*3,(1/2-rand())*2*pi,(1/2-rand(1,7))*2*pi]; % random config
            
            % set KMR config
            vrep_setKMRConfiguration(obj.vrep, obj.clientID, conf(1:3), obj.B_handle);
            
            % set iiwa config
            vrep_setIiwaConfiguration(obj.vrep, obj.clientID, conf(4:10), obj.joints_handles);
            
            % set full system config
            vrep_setFullSystemConfiguration(obj.vrep, obj.clientID, -conf, obj.B_handle, obj.joints_handles);
            
            
            
            % get distances from collision and autocollision
            distances = vrep_getDistancesToObstacles(obj.clientID, obj.vrep, obj.distances_handles);
        end
        
        
        
        
        
        
        %---------------------
        % get distances
        %---------------------
        function distances_values = getDistancesAndNames(obj)
            % conf in m and rad
            distances_values = vrep_getDistancesToObstacles(obj.clientID, obj.vrep, obj.distances_handles);
        end
        
        
        
        
        
        %---------------------
        % set KMR config
        %---------------------
        function setKMRConfiguration(obj, conf)
            % conf in m and rad
            vrep_setKMRConfiguration(obj.vrep, obj.clientID,conf,obj.B_handle);
        end
        
        
        
        
        
        
        
        %---------------------
        % set iiwa config
        %---------------------
        function setIiwaConfiguration(obj, conf)
            % conf in rad
            vrep_setIiwaConfiguration(obj.vrep, obj.clientID,conf,obj.joints_handles);
        end
        
        
        
        
        
        
        %---------------------
        % set full system config
        %---------------------
        function setFullSystemConfiguration(obj, conf)
            % conf in m and rad
            vrep_setFullSystemConfiguration(obj.vrep, obj.clientID,conf,obj.B_handle,obj.joints_handles);
        end
        
        
        
    end
end