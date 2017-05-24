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
        m_EE; % mass EE
        cog_EE; % center of gravity end effector
        
        
        %distance object
        distances_handles;
        distances_names;
        
        %tasks positions, handles and names
        tasks;
        tasks_handles
        tasks_names;
        
        %proximity sensors handles
        proximitySensors_handles
        proximitySensors_names
        
        
        
        
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
                
                % retrieve load data
                [obj.cog_EE, obj.m_EE, ret] = vrep_getLoadData(obj.clientID,obj.vrep);
                if ret == 0
                    disp('    -iiwa load data retrieved');
                else
                    disp('    -error on load data retrieval');
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
                
                % retrieve proximity Sensors handles and names
                [obj.proximitySensors_handles, obj.proximitySensors_names, ret] = vrep_getProximitySensorsHandles(obj.clientID,obj.vrep);
                if ret == 0
                    disp('    -proximity sensors handles retrieved');
                else
                    disp('    -error on proximity sensors handles retrieval');
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
            % in meter
            distances_values = vrep_getDistancesToObstacles(obj.clientID, obj.vrep, obj.distances_handles);
%             pause(0.001);
        end
        
        
        
        
        %---------------------
        % get entanglement Distances (if the meshes are colliding, only
        % proximity sensors mesuring the inside of the mesh will be able to
        % give an idea of the "distance of entanglement"
        %---------------------
        function [ detectionStates, detectedPoints, detectedObjectHandles, detectedSurfacesNormalVector ] = getProximitySensorsMeasurements(obj)
            % in meter
            [ detectionStates, detectedPoints, detectedObjectHandles, detectedSurfacesNormalVector ] = vrep_readProximitySensors(obj.clientID,obj.vrep, obj.proximitySensors_handles);
        end
        
        
        
        %---------------------
        % retrieve distances to obstacles and when in collision, retrieves
        % the associated proximity sensor mesurement
        %---------------------
        function [fullDistances_values] = getFullDistances(obj)
            % in meter
            % for each distance fetched, test wether it's greater than 0. If
            % not , retrieve the associated proximity sensor(s) and process
            % their values.
            
            for i = 1:3 %to be sure we have it....
                distances_values = getDistancesToObstacles(obj); %to be sure we have it....
            end
            
            [ detectionStates, detectedPoints, detectedObjectHandles, detectedSurfacesNormalVector ] = getProximitySensorsMeasurements(obj);
            fullDistances_values = zeros(length(distances_values),1);
            
            
            % KMR distances
                % to scene
                if distances_values(10) < eps % if distance corresponding to KMR-scene is 0 

                    % find proximity sensor indices corresponding to KMR-scene
                    % measurements
                    range_length = 1.21/2; % m
                    range_width = 0.73/2; % m
                    detectedPoints_KMR = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_KMR_scene'))),:); % retrieve all distances readings from KMR related proximity sensors
                    [closest_d, ind_closest] = min(detectedPoints_KMR(:,1).^2 + detectedPoints_KMR(:,2).^2 + detectedPoints_KMR(:,3).^2); 
                    
                    detectedPoint_KMR = detectedPoints_KMR(ind_closest,:);

                    d_entanglement_x = range_length - abs(detectedPoint_KMR(1));
                    d_entanglement_y = range_width - abs(detectedPoint_KMR(2));

                    fullDistances_values(10) = -min([d_entanglement_x,d_entanglement_y]);

                else
                    fullDistances_values(10) = distances_values(10);
                end
                
                % to link3
                if distances_values(1) < eps % if distance corresponding to KMR-link3 is 0 
                    % find proximity sensor indices corresponding to KMR-link3 measurement
                    detectedPoint_KMR = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_KMR_link3'))),:); 
                    
                    char_length = sqrt((1.2100/2).^2+(0.703/2).^2+(0.73/2).^2); %characteristic length of the KMR (took the diagonal of the parallelepiped to be sure it always returns a positive number)
                    fullDistances_values(1) = -(char_length - norm(detectedPoint_KMR));
                else
                    fullDistances_values(1) = distances_values(1);
                end
            
                % to link4
                if distances_values(2) < eps % if distance corresponding to KMR-link4 is 0 
                    % find proximity sensor indices corresponding to KMR-link4 measurement
                    detectedPoint_KMR = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_KMR_link4'))),:); 
                    
                    char_length = sqrt((1.2100/2).^2+(0.703/2).^2+(0.73/2).^2); %characteristic length of the KMR (took the diagonal of the parallelepiped to be sure it always returns a positive number)
                    fullDistances_values(2) = -(char_length - norm(detectedPoint_KMR));
                else
                    fullDistances_values(2) = distances_values(2);
                end
            
                % to link5
                if distances_values(3) < eps % if distance corresponding to KMR-link5 is 0 
                    % find proximity sensor indices corresponding to KMR-link5 measurement
                    detectedPoint_KMR = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_KMR_link5'))),:); 
                    
                    char_length = sqrt((1.2100/2).^2+(0.703/2).^2+(0.73/2).^2); %characteristic length of the KMR (took the diagonal of the parallelepiped to be sure it always returns a positive number)
                    fullDistances_values(3) = -(char_length - norm(detectedPoint_KMR));
                else
                    fullDistances_values(3) = distances_values(3);
                end
            
                % to link6
                if distances_values(4) < eps % if distance corresponding to KMR-link6 is 0 
                    % find proximity sensor indices corresponding to KMR-link6 measurement
                    detectedPoint_KMR = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_KMR_link6'))),:); 
                    
                    char_length = sqrt((1.2100/2).^2+(0.703/2).^2+(0.73/2).^2); %characteristic length of the KMR (took the diagonal of the parallelepiped to be sure it always returns a positive number)
                    fullDistances_values(4) = -(char_length - norm(detectedPoint_KMR));
                else
                    fullDistances_values(4) = distances_values(4);
                end
                
                % to linkFlange
                if distances_values(5) < eps % if distance corresponding to KMR-linkFlange is 0 
                    % find proximity sensor indices corresponding to KMR-linkFlange measurement
                    detectedPoint_KMR = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_KMR_linkFlange'))),:); 
                    
                    char_length = sqrt((1.2100/2).^2+(0.703/2).^2+(0.73/2).^2); %characteristic length of the KMR (took the diagonal of the parallelepiped to be sure it always returns a positive number)
                    fullDistances_values(5) = -(char_length - norm(detectedPoint_KMR));
                else
                    fullDistances_values(5) = distances_values(5);
                end
                
            % EE distances
                % to KMR
                if distances_values(6) < eps % if distance corresponding to EE-KMR is 0 
                    % find proximity sensor indices corresponding to EE-KMR measurement
                    detectedPoint_EE = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_EE_KMR'))),:); 
                    
                    char_length = sqrt((0.25/2).^2+(0.1/2).^2+(0.05/2).^2); %characteristic length of the EE (took the diagonal of the parallelepiped to be sure it always returns a positive number)
                    fullDistances_values(6) = -(char_length - norm(detectedPoint_EE));
                else
                    fullDistances_values(6) = distances_values(6);
                end
                
                % to link0
                if distances_values(7) < eps % if distance corresponding to EE-link0 is 0 
                    % find proximity sensor indices corresponding to EE-link0 measurement
                    detectedPoint_EE = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_EE_link0'))),:); 
                    
                    char_length = sqrt((0.25/2).^2+(0.1/2).^2+(0.05/2).^2); %characteristic length of the EE (took the diagonal of the parallelepiped to be sure it always returns a positive number)
                    fullDistances_values(7) = -(char_length - norm(detectedPoint_EE));
                else
                    fullDistances_values(7) = distances_values(7);
                end
                
                % to link1
                if distances_values(8) < eps % if distance corresponding to EE-link1 is 0 
                    % find proximity sensor indices corresponding to EE-link1 measurement
                    detectedPoint_EE = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_EE_link1'))),:); 
                    
                    char_length = sqrt((0.25/2).^2+(0.1/2).^2+(0.05/2).^2); %characteristic length of the EE (took the diagonal of the parallelepiped to be sure it always returns a positive number)
                    fullDistances_values(8) = -(char_length - norm(detectedPoint_EE));
                else
                    fullDistances_values(8) = distances_values(8);
                end
                
                % to link2
                if distances_values(9) < eps % if distance corresponding to EE-link2 is 0 
                    % find proximity sensor indices corresponding to EE-link2 measurement
                    detectedPoint_EE = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_EE_link2'))),:); 
                    
                    char_length = sqrt((0.25/2).^2+(0.1/2).^2+(0.05/2).^2); %characteristic length of the EE (took the diagonal of the parallelepiped to be sure it always returns a positive number)
                    fullDistances_values(9) = -(char_length - norm(detectedPoint_EE));
                else
                    fullDistances_values(9) = distances_values(9);
                end
                
            % scene distances
                % link0 to scene
                if distances_values(11) < eps % if distance corresponding to link0_scene is 0 
                    % find proximity sensor indices corresponding to link0_scene measurement
                    detectedPoint_link = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_link0_scene'))),:); 
                    
                    char_length = sqrt((0.1500/2).^2 + (0.14/2).^2); %characteristic length of the link (took the distance going from the center of the cylinder to the edge of the base of the cylinder to be sure it always returns a positive number)
                    fullDistances_values(11) = -(char_length - norm(detectedPoint_link));
                else
                    fullDistances_values(11) = distances_values(11);
                end
                
                % link1 to scene
                if distances_values(12) < eps % if distance corresponding to link1_scene is 0 
                    % find proximity sensor indices corresponding to link1_scene measurement
                    detectedPoint_link = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_link1_scene'))),:); 
                    
                    char_length = sqrt((0.2900/2).^2 + (0.09/2).^2); %characteristic length of the link (took the distance going from the center of the cylinder to the edge of the base of the cylinder to be sure it always returns a positive number)
                    fullDistances_values(12) = -(char_length - norm(detectedPoint_link));
                else
                    fullDistances_values(12) = distances_values(12);
                end
                
                % link2 to scene
                if distances_values(13) < eps % if distance corresponding to link2_scene is 0 
                    % find proximity sensor indices corresponding to link2_scene measurement
                    detectedPoint_link = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_link2_scene'))),:); 
                    
                    char_length = sqrt((0.2900/2).^2 + (0.09/2).^2); %characteristic length of the link (took the distance going from the center of the cylinder to the edge of the base of the cylinder to be sure it always returns a positive number)
                    fullDistances_values(13) = -(char_length - norm(detectedPoint_link));
                else
                    fullDistances_values(13) = distances_values(13);
                end
                
                % link3 to scene
                if distances_values(14) < eps % if distance corresponding to link3_scene is 0 
                    % find proximity sensor indices corresponding to link3_scene measurement
                    detectedPoint_link = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_link3_scene'))),:); 
                    
                    char_length = sqrt((0.2900/2).^2 + (0.09/2).^2); %characteristic length of the link (took the distance going from the center of the cylinder to the edge of the base of the cylinder to be sure it always returns a positive number)
                    fullDistances_values(14) = -(char_length - norm(detectedPoint_link));
                else
                    fullDistances_values(14) = distances_values(14);
                end
                
                % link4 to scene
                if distances_values(15) < eps % if distance corresponding to link4_scene is 0 
                    % find proximity sensor indices corresponding to link4_scene measurement
                    detectedPoint_link = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_link4_scene'))),:); 
                    
                    char_length = sqrt((0.2700/2).^2 + (0.09/2).^2); %characteristic length of the link (took the distance going from the center of the cylinder to the edge of the base of the cylinder to be sure it always returns a positive number)
                    fullDistances_values(15) = -(char_length - norm(detectedPoint_link));
                else
                    fullDistances_values(15) = distances_values(15);
                end
                
                % link5 to scene
                if distances_values(16) < eps % if distance corresponding to link5_scene is 0 
                    % find proximity sensor indices corresponding to link5_scene measurement
                    detectedPoint_link = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_link5_scene'))),:); 
                    
                    char_length = sqrt((0.2600/2).^2 + (0.08/2).^2); %characteristic length of the link (took the distance going from the center of the cylinder to the edge of the base of the cylinder to be sure it always returns a positive number)
                    fullDistances_values(16) = -(char_length - norm(detectedPoint_link));
                else
                    fullDistances_values(16) = distances_values(16);
                end
                
                % link6 to scene
                if distances_values(17) < eps % if distance corresponding to link6_scene is 0 
                    % find proximity sensor indices corresponding to link6_scene measurement
                    detectedPoint_link = detectedPoints(not(cellfun('isempty', strfind(obj.proximitySensors_names, 'proxSensor_link6_scene'))),:); 
                    
                    char_length = sqrt((0.1650/2).^2 + (0.07/2).^2); %characteristic length of the link (took the distance going from the center of the cylinder to the edge of the base of the cylinder to be sure it always returns a positive number)
                    fullDistances_values(17) = -(char_length - norm(detectedPoint_link));
                else
                    fullDistances_values(17) = distances_values(17);
                end
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