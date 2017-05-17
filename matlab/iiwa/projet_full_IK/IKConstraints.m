function [c,ceq] = IKConstraints(x,vrep_store,maxDeflections,forceCapa)


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

%% parameters setting
% hardcoded args
H_b = 703.07; % platform height (mm) {TODO : hardcode}
rmin = 410.3656905736638; % minimum distance between shoulder and wrist ( = sqrt(400^2+420^2-2*400*420*cos(60*pi/180)) )
rmax = 820; % maximum distance between shoulder and wrist ( = 420 + 400 )
distance_collision_threshold = 0.5; % (m) system can't get closer to an obstacle than distance_obstacle_threshold 
distance_autocollision_threshold = 0.5; % (m) same threshold but for autocollision
default_constraint_value = 1000;


% extraction from inputs
x_b = x(1); % 1st coordinate of the robot base position in ref frame world
y_b = x(2); % 2nd coordinate of the robot base position in ref frame world
theta_b = setAnglesBetweenMinusPiAndPi(x(3)); % angle of the robot base position around z_w 
betas = setAnglesBetweenMinusPiAndPi(x(3:end)); % redundancy (/swivel) angles for each task (between -pi and pi)
ndistances = length(vrep_store.distances_handles);



tasks = vrep_store.tasks;
ntasks = length(tasks);

 
c = zeros(ntasks +      ntasks       +     1      +   ntasks*(ndistances-1)    ,1);


%         reach   minDistBetaPerTask   dKMR_scene     severalDistancesPerTasks   
% reach : each task has to be reachable
% jlimits : each task has to enable at least one config within joint limits
% dKMR_scene : distance of the mobile platform to the closest obstacle (the platform is still)
% severalDistancesPerTasks :  all other distances have to be checked for each task
% ndeflection : in our case, the deflection is the same for each of the 8 available configs, so one per deflection
% nforceCapa : in our case, the forceCapa is the same for each of the 8 available configs, so one per forceCapa

%% reachability  ------->  c(1:ntasks)

% compute H_W_B <=> robot base ref frame in world ref frame
ct = cos(theta_b);
st = sin(theta_b);
H_W_B = [ct, -st, 0, x_b*1000; ...
         st,  ct, 0, y_b*1000; ...
         0 ,   0, 1, H_b; ...
         0 ,   0, 0,   1];
H_B_W = inverseTransformation(H_W_B); %TODO, hardcode this

DeltaEE = vrep_store.tcp_offset; % get TCP offset
H_tcp_7 = inverseTransformation(peaZYX_to_transformation(DeltaEE)); %express it as a transformation and matrix invert it  
p_7_6 = [0;0;-126;1]; % normal flange -------------------------    /!\flange dependant!!!!!!!
p_tcp_6 = H_tcp_7*p_7_6; % wrist position seen from tcp . TODO hardcode this
p_B_shoulder = [0;0;360;1]; % shoulder position seen from robot base . TODO hardcode this


% alreadyBeenHere = true;
% for ind_task = 1:ntasks
% % for ind_task = 1:10
%     
% %% reachability 
% 
% %     H_W_ti = tasks.getTask(i).getTransf();
%     H_W_ti = peaZYX_to_transformation(tasks{ind_task});
%     p_B_wrist = H_B_W*H_W_ti*p_tcp_6; % compute hypothetical wrist position seen from robot base
%     d_reach = norm(p_B_wrist(1:3)-p_B_shoulder(1:3)); % compute distance between wrist and shoulder
%     c(0+ind_task) = - ( (d_reach<(rmin+rmax)/2)*(d_reach-rmin) + (d_reach>=(rmin+rmax)/2)*(rmax-d_reach) ); % c is negative when d€[rmin,rmax] and positive when out. The function is continuous, prolongable in (rmin+rmax)/2 and reaches its minimum there.
% end
% 
% for ind_task = 1:ntasks
% % for ind_task = 1:10
%     H_W_ti = peaZYX_to_transformation(tasks{ind_task});
%     if (any(c(1:ntasks) >= sqrt(eps)))
%         % there is no point checking other constraints or trying to comply
%         % with them while the poses are not reachable so all other
%         % constraints are set to zero and the config of the iiwa is set to
%         % zero position
%         if alreadyBeenHere
%             alreadyBeenHere = false; % if
%             % joint limits check
% %             c(ntasks+ind_task) = -epsilon;
% %             vrep_store.setIiwaConfiguration([0,0,0,0,0,0,0]);
% %             % autocollision
% %             c((2*ntasks + (ind_task-1)*ndistances+1):(2*ntasks + (ind_task-1)*ndistances+9)) = -epsilon; %
% %             % collisions
% %             c((2*ntasks + (ind_task-1)*(ndistances)+10):(2*ntasks + (ind_task-1)*(ndistances)+ndistances)) = -epsilon;
%             
% %             c(ntasks+1:end) = -eps;
%             c(ntasks+1:end) = rand();
%         end
%         
%     else
%         %% analytical IK + check limits
%         H_B_tcp = H_B_W*H_W_ti;
%         %     transformation_to_peaZYX(H_W_ti)
%         [q, D_Phi, indD_Phi] = computeIKIiwa2(DeltaEE,transformation_to_peaZYX(H_B_tcp),betas(ind_task));
%         if imag(q) == 1
%             c(ntasks:end) = -eps;
%             break;
%         end
%         
%         c(ntasks+ind_task) = D_Phi*180/pi;
%         
%         %% distance checks
%         
%         vrep_store.setIiwaConfiguration(q(indD_Phi,1:7));
%         distances_values = vrep_store.getDistancesToObstacles();
%         
%         if false
%             disp(['task #',num2str(ind_task)])
%             for i=1:ndistances
%                 disp([vrep_store.distances_names(i), num2str(distances_values(i))]) 
%             end
%         end
%         
%         % autocollisions
%         c((2*ntasks + (ind_task-1)*ndistances+1):(2*ntasks + (ind_task-1)*ndistances+9)) = -(distances_values(1:9) - rand()*distance_autocollision_threshold)*1000; % (in millimeter) autocollision distances. constraints should be negative when it's alright and +distance_autocollision_threshold when it's not
%         
%         % collisions
%         c((2*ntasks + (ind_task-1)*(ndistances)+10):(2*ntasks + (ind_task-1)*(ndistances)+ndistances)) = -(distances_values(10:ndistances) - rand()*distance_collision_threshold)*1000; % (in millimeter) distances to obstacles constraints should be negative when it's alright and +distance_threshold when it's not
% %         if any(c>1)
% %             c
% %         end
%     end
%     
% end






    function d_reach = distanceToBeingReachable()
        p_B_wrist = H_B_W*H_W_ti*p_tcp_6; % compute required wrist position (seen from robot base) to perform task i
        d_sw = norm(p_B_wrist(1:3)-p_B_shoulder(1:3)); % compute distance between wrist and shoulder
        d_reach = - ( (d_sw<(rmin+rmax)/2)*(d_sw-rmin) + (d_sw>=(rmin+rmax)/2)*(rmax-d_sw) );
    end
    
    function d_col = distancesToCollisions()
        distances_values = vrep_store.getFullDistances();
        distances_autocol = distances_values(1:9);
        distances_col = distances_values(11:17);
        
        if any(distances_values == 0)
            bob=1;
        end
        
        distances_autocol(distances_autocol < eps) = -distance_autocollision_threshold*rand(); % randomise if the distance is 0
        distances_col(distances_col < eps) = -distance_collision_threshold*rand(); % randomise if the distance is 0
        
        
        
        d_col = -[distances_autocol;distances_col]*1000;
    end





% set KMR config
vrep_store.setKMRConfiguration([x_b,y_b,theta_b]);

% retrieve KMR distance to scene and store it in c
% distances_values = vrep_store.getDistancesToObstacles(); % former : doesn't take into account entanglement distances
distances_values = vrep_store.getFullDistances(); % takes into account entanglement distances
distance_KMR_scene = distances_values(10);
if distances_values(10) == 0
    bob=1;
end

if distances_values(10) < 0
    bob=1; 
end
distance_KMR_scene(abs(distance_KMR_scene) < 0.0001 ) = -distance_collision_threshold*rand();
c(end) = -distance_KMR_scene*1000;


for ind_task = 1:ntasks
    H_W_ti = peaZYX_to_transformation(tasks{ind_task});
    N = 1+1+(ndistances-1);
    
    d_reach = distanceToBeingReachable(); 
    c(N*(ind_task-1) + 1) = d_reach;
    
    if d_reach < -10 % mm
        H_B_tcp = H_B_W*H_W_ti;
        [q, D_Phi, indD_Phi] = computeIKIiwa2(DeltaEE,transformation_to_peaZYX(H_B_tcp),betas(ind_task));
        c(N*(ind_task-1) + 2) = D_Phi*180/pi; % in degrees
        
        % set iiwa configuration
        vrep_store.setIiwaConfiguration(q(indD_Phi,1:7));
        
        % retrieve and store distances to auto collision and to collision
        c(N*(ind_task-1) + 3: N*ind_task) = distancesToCollisions(); 
    else
        c(N*(ind_task-1) + 2 : N*ind_task) = 0; %default_constraint_value*(1/2-rand()); % if task isn't reachable, then set a positive value to all other remaining constraints associated to the task
    end
    
    
end



% ceq = abs(c(1:ntasks)) + c(1:ntasks);
x;
c;










































ceq = [];

end

