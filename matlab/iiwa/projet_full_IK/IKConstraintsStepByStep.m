function [c,ceq] = IKConstraintsStepByStep(x,vrep_store, constraintsToCheck, maxDeflections,forceCapa)


% x                     are the positions in redundancy space
% tasks                 gathers all the tasks that are to be
%                       performed by the robot without moving the mobile
%                       plateform. Along the pose (expressed in the world
%                       reference frame), a force vector is given to
%                       describe the kind of interaction and provide
%                       information for the deflection and forceCapacity
%                       solvers.
% obstacles             gathers the geometry of obstacle objects
% maxDeflections        gathers the maximal accepted deflections at the end
%                       effector. Multiple directions can be contrained.
%                       Rotations and translation must be treated
%                       separately
% Additional remarks : joint limits will be hardcoded
%
% Goal : Provide non-linear contraints for matlab fmincon solver. IK is
% resolved analytically out of chosen positions in redundancy space (x) and
% a number of tasks (tasks). 
% 
% 1) For each task, reachability is checked 
% 2) For each task, an analytical IK is then performed to compute q
% 3) For each task, the computed q is checked to lie within the joint
% limits
% 4) For each task, the FK of each link is computed and a computation of
% the distance to obstacles is performed
% 5) For each task, the deflections are computed 
% 6) For each task, the force Capacity is computed


% x = [x_b, y_b, theta_b, swivel_1, swivel_2, ... , swivel_m] where x_b and
% y_b are the x_w and y_w coordinates (in the world reference frame) of the
% base of the robot and theta_b is the angle taken by the plateform about
% z_w
%
% tasks = {task_1, task_2, ... task_m} where task_m0 contains at least the
% associated pose_m0 where the task occurs and wrench_0 which describes the
% force interaction occuring at task_m0 with the environment. 
%   the task is given with ZYX euler convention
%
% maxDeflections = {deflection_1, deflection_2, ... , deflection_d}
% where deflection_d0 contains a vector along which the deflection is
% limited to the norm of the vector (I don't know how to integrate
% rotative deflections though)
%
% forceCapa = {forceCapa_1, forceCapa_2, ... , forceCapa_f}
% where forceCapa_f0 contains a vector along which the force capacity must
% be allowed

if true % parameters settings and geometric computations
        H_b = 703.07; % platform height (mm) {TODO : hardcode}
        rmin = 410.3656905736638; % minimum distance between shoulder and wrist ( = sqrt(400^2+420^2-2*400*420*cos(60*pi/180)) )
        rmax = 820; % maximum distance between shoulder and wrist ( = 420 + 400 )
        distance_collision_threshold = 0.5; % (m) system can't get closer to an obstacle than distance_obstacle_threshold
        distance_autocollision_threshold = 0.5; % (m) same threshold but for autocollision
        ndistances = length(vrep_store.distances_handles);
        
        % extraction from inputs
        x_b = x(1); % 1st coordinate of the robot base position in ref frame world
        y_b = x(2); % 2nd coordinate of the robot base position in ref frame world
        theta_b = setAnglesBetweenMinusPiAndPi(x(3)); % angle of the robot base position around z_w
        betas = setAnglesBetweenMinusPiAndPi(x(3:end)); % redundancy (/swivel) angles for each task (between -pi and pi)
        
        tasks = vrep_store.tasks;
        ntasks = length(tasks);

        % compute H_W_B <=> robot base ref frame in world ref frame
        ct = cos(theta_b);
        st = sin(theta_b);
        H_W_B = [ct, -st, 0, x_b*1000; ...
            st,  ct, 0, y_b*1000; ...
            0 ,   0, 1, H_b; ...
            0 ,   0, 0,   1];
        H_B_W = inverseTransformation(H_W_B); %TODO, hardcode this
        
        DeltaEE = vrep_store.tcp_offset; % get TCP offset
        H_tcp_7 = inverseTransformation(peaZYX_to_transformation(DeltaEE)); % express it as a transformation and matrix invert it
        p_7_6 = [0;0;-126;1]; % vrep flange -------------------------    /!\flange dependant!!!!!!!
        p_tcp_6 = H_tcp_7*p_7_6; % wrist position seen from tcp .
        p_B_shoulder = [0;0;360;1]; % shoulder position seen from robot base .
end
    



function d_reach = distanceToBeingReachable()
        p_B_wrist = H_B_W*H_W_ti*p_tcp_6; % compute required wrist position (seen from robot base) to perform task i
        d_sw = norm(p_B_wrist(1:3)-p_B_shoulder(1:3)); % compute distance between wrist and shoulder
        d_reach = - ( (d_sw<(rmin+rmax)/2)*(d_sw-rmin) + (d_sw>=(rmin+rmax)/2)*(rmax-d_sw) );
        end


function d_col = distancesToCollisions()
    distances_values = vrep_store.getFullDistances();
    distances_autocol = distances_values(1:9);
    distances_col = distances_values(11:17);
    distances_autocol(distances_autocol < eps) = -distance_autocollision_threshold*rand(); % randomise if the distance is 0
    distances_col(distances_col < eps) = -distance_collision_threshold*rand(); % randomise if the distance is 0

    d_col = -[distances_autocol;distances_col]*1000;
end


% checkReachability = any(constraintsToCheck == 1);
checkReachability = true; % reachibility is always to be sought otherwise other constraints make no sense (except maybe KMRCollision. discussable...)
checkKMRCollisions = any(constraintsToCheck == 2);
checkOtherCollisions = any(constraintsToCheck == 3);
checkJointsLimits = any(constraintsToCheck == 4);

constraintsCell = cell(4,1);
% set KMR config
vrep_store.setKMRConfiguration([x_b,y_b,theta_b]);
pause(0.1)
    
if checkReachability % check reachability
    
    
    for ind_task = 1:ntasks % compute d_reach
        H_W_ti = peaZYX_to_transformation(tasks{ind_task});
        d_reach_taski = distanceToBeingReachable();
        if ind_task == 1
            d_reach = d_reach_taski;
        else
            d_reach = max(d_reach_taski,d_reach);
        end
    end
    constraintsCell{1} = d_reach*1000;
end



if checkKMRCollisions % get KMR distance to scene
    
    
    % retrieve KMR distance to scene and store it in c
    distances_values = vrep_store.getFullDistances(); % takes into account entanglement distances
    distance_KMR_scene = distances_values(10)*1000; % mm
    distance_KMR_scene(abs(distance_KMR_scene) < 0.0001 ) = -distance_collision_threshold*rand();
    constraintsCell{2} = -distance_KMR_scene;
end



if checkOtherCollisions && checkJointsLimits
    d_col = zeros((ndistances-1)*ntasks,1);
    d_jlimits = zeros(ntasks,1);
    
    if constraintsCell{1} <= 0 % if reachability is respected
        for ind_task = 1:ntasks % get all other distances to obstacles
            H_W_ti = peaZYX_to_transformation(tasks{ind_task});
            
            H_B_tcp = H_B_W*H_W_ti;
            [q, D_Phi, indD_Phi] = computeIKIiwa3(DeltaEE,transformation_to_peaZYX(H_B_tcp),betas(ind_task));
            d_jlimits(ind_task) = D_Phi*180/pi; % in degrees
            
            % set iiwa configuration
            vrep_store.setIiwaConfiguration(q(indD_Phi,1:7));
            
            % retrieve and store distances to auto collision and to collision
            d_col((ind_task-1)*(ndistances-1)+1 : ind_task*(ndistances-1)) = distancesToCollisions();
            
        end
    end
    
    constraintsCell{3} = d_col;
    constraintsCell{4} = d_jlimits;
    
elseif checkOtherCollisions && ~checkJointsLimits
    d_col = zeros((ndistances-1)*ntasks,1);
    if constraintsCell{1} <= 0 % if reachability is respected
        for ind_task = 1:ntasks % get all other distances to obstacles
            H_W_ti = peaZYX_to_transformation(tasks{ind_task});
            
            H_B_tcp = H_B_W*H_W_ti;
            [q] = computeIKIiwa1(DeltaEE,transformation_to_peaZYX(H_B_tcp),betas(ind_task));
            
            % set iiwa configuration
            vrep_store.setIiwaConfiguration(q(1,1:7));
            
            % retrieve and store distances to auto collision and to collision
            d_col((ind_task-1)*(ndistances-1)+1 : ind_task*(ndistances-1)) = distancesToCollisions();
            
        end
    end
    constraintsCell{3} = d_col;
    
elseif ~checkOtherCollisions && checkJointsLimits
    d_jlimits = zeros(ntasks,1);
    if constraintsCell{1} <= 0 % if reachability is respected
        for ind_task = 1:ntasks % get all other distances to obstacles
            H_W_ti = peaZYX_to_transformation(tasks{ind_task});
            
            H_B_tcp = H_B_W*H_W_ti;
            [q, D_Phi, indD_Phi] = computeIKIiwa3(DeltaEE,transformation_to_peaZYX(H_B_tcp),betas(ind_task));
            
            % set iiwa configuration
            vrep_store.setIiwaConfiguration(q(1,1:7));
            
            d_jlimits(ind_task) = D_Phi*180/pi; % in degrees
        end
    end
    constraintsCell{4} = d_jlimits;
    
end




c = double([constraintsCell{1};constraintsCell{2};constraintsCell{3};constraintsCell{4}]);
figure(2)
plot(c,'o')
ceq = [];

end
