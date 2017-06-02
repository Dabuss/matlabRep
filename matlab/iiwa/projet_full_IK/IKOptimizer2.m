function [v] = IKOptimizer2(x,vrep_store, multfactor)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


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

% extraction from inputs
x_b = x(1); % 1st coordinate of the robot base position in ref frame world
y_b = x(2); % 2nd coordinate of the robot base position in ref frame world
theta_b = setAnglesBetweenMinusPiAndPi(x(3)); % angle of the robot base position around z_w 




tasks = vrep_store.tasks;
ntasks = length(tasks);

 
c = zeros(ntasks,1);


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


function d = distanceToBeingReachable()
    p_B_wrist = H_B_W*H_W_ti*p_tcp_6; % compute required wrist position (seen from robot base) to perform task i
    d_sw = norm(p_B_wrist(1:3)-p_B_shoulder(1:3)); % compute distance between wrist and shoulder
    d = - ( (d_sw<(rmin+rmax)/2)*(d_sw-rmin) + (d_sw>=(rmin+rmax)/2)*(rmax-d_sw) );
end

d_reach = zeros(ntasks,1);
for ind_task = 1:ntasks
    H_W_ti = peaZYX_to_transformation(tasks{ind_task});

    
    d_reach(ind_task) = distanceToBeingReachable(); 
    c(ind_task) = d_reach(ind_task);
end


c(c>=0) = c(c>=0)*multfactor;
v = std(c);
% v = abs(min(c)-max(c));
% v = sum(c.^2);
bob=1;
end

