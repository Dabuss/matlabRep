classdef vrepStore_miiwa < handle
    
    properties
        %vrep specific
        vrep;
        clientID;
        
        %joint and Base
        joints_handles;
        joints_names;
        B_handle;
        
        % end effector
        tcp_handle;
        tcp_offset;
        
        %distance object
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
                [obj.joints_handles, obj.joints_names, ret] = vrep_getJointsHandles(obj.clientID,obj.vrep);
                if ~isempty(obj.joints_handles)
                    disp('    -iiwa joints handles retrieved');
                else
                    disp('    -error on iiwa joints handles retrieval');
                end
                
                
                % retrieve KMR dummy handle
                [ret,obj.B_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'B',obj.vrep.simx_opmode_blocking);
                if ret == 0
                    disp('    -KMR Base handle retrieved');
                else
                    disp('    -error on KMR Base handle retrieval');
                end
                
                 % retrieve tcp handle
                [ret, obj.tcp_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'TCP',obj.vrep.simx_opmode_blocking);
                if ret == 0
                    disp('    -iiwa tcp handle retrieved');
                else
                    disp('    -error on tcp handle retrieval');
                end
                
                % retrieve tcp offset
                [obj.tcp_offset, ret] = vrep_getTcpOffset(obj.clientID,obj.vrep, obj.tcp_handle);
                if ret == 0
                    disp('    -iiwa tcp offset retrieved');
                else
                    disp('    -error on tcp offset retrieval');
                end
                
                % retrieve distances handles
                [obj.distances_handles,obj.distances_names, ret] = vrep_getDistancesHandles(obj.clientID,obj.vrep);
                if ret == 0
                    disp('    -distances handles retrieved');
                else
                    disp('    -error on distances handles retrieval');
                end
                
                % retrieve tasks handles
                [obj.tasks_handles,obj.tasks_names, ret] = vrep_getTasksHandles(obj.clientID,obj.vrep);
                if ~isempty(obj.tasks_handles)
                    disp('    -tasks handles retrieved');
                else
                    disp('    -error on tasks handles retrieval');
                end
                
                % get tasks EulerZYX poses
                [obj.tasks,ret] = vrep_getTasksEulerZYXPoses(obj.clientID, obj.vrep, obj.tasks_handles);
                if ~isempty(obj.tasks)
                    disp('    -tasks poses retrieved');
                else
                    disp('    -error on tasks poses retrieval');
                end
                
                
                
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
        function distances_values = getDistancesToObstacles(obj)
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
        
        
        %---------------------
        % generate random configuration for the entire system
        %---------------------
        function conf = getRandomValidConfig(obj)
            conf = [(1/2-rand(1,2))*3,...       % x_b and y_b
                    (1/2-rand())*2*pi,...       % theta_b
                    (1/2-rand())*170*pi/180,... % q1
                    (1/2-rand())*120*pi/180,... % q2
                    (1/2-rand())*170*pi/180,... % q3
                    (1/2-rand())*120*pi/180,... % q4
                    (1/2-rand())*170*pi/180,... % q5
                    (1/2-rand())*120*pi/180,... % q6
                    (1/2-rand())*175*pi/180];   % q7
        end
        
    end
end