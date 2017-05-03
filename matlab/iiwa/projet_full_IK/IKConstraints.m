function [c,ceq] = IKConstraints(x,tasks,obstacles,maxDeflections,forceCapa)


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
%
% obstacles {obstacle_1,obstacle_2, ... , obstacle_o} where obstacle_o0 is
% a structured way of describing the obstacles
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
H_b = 1000; % platform height (mm) {TODO : hardcode}
rmin = 410.3656905736638; % minimum distance between shoulder and wrist ( = sqrt(400^2+420^2-2*400*420*cos(60*pi/180)) )
rmax = 820; % maximum distance between shoulder and wrist ( = 420 + 400 )

% extraction from inputs
x_b = x(1); % 1st coordinate of the robot base position in ref frame world
y_b = x(2); % 2nd coordinate of the robot base position in ref frame world
theta_b = setAnglesBetweenMinusPiAndPi(x(3)); % angle of the robot base position around z_w 
betas = setAnglesBetweenMinusPiAndPi(x(3:end)); % redundancy (/swivel) angles for each task (between -pi and pi)

% ntasks = tasks.getLength();
ntasks = length(tasks);
% ndeflection = maxDeflections.getLength();
% nforceCapa = forceCapa.getLength();


% c = zeros(1, ntasks + 7*2*ntasks + 1        + ndeflection + nforceCapa); 
c = zeros(ntasks + ntasks,1);
%            reach   7*2*jlimits   dobstacles    deflection   forceCapa
% reach : each task has to be reachable
% jlimits : each task has to enable at least one config within joint limits
% dobstacle : distance of the mobile platform to the closest obstacle
% ndeflection : in our case, the deflection is the same for each of the 8 available configs, so one per deflection
% nforceCapa : in our case, the forceCapa is the same for each of the 8 available configs, so one per forceCapa

%% reachability  ------->  c(1:ntasks)

% compute H_W_B <=> robot base ref frame in world ref frame
ct = cos(theta_b);
st = sin(theta_b);
H_W_B = [ct, -st, 0, x_b; ...
         st,  ct, 0, y_b; ...
         0 ,   0, 1, H_b; ...
         0 ,   0, 0,   1];
H_B_W = inverseTransformation(H_W_B); %TODO, hardcode this

DeltaEE = [0,0,0,0,0,0];
H_tcp_7 = inverseTransformation(poseEulerZYXToTransf(DeltaEE));
p_7_6 = [0;0;-126;1]; % normal flange -------------------------    /!\flange dependant!!!!!!!
p_tcp_6 = H_tcp_7*p_7_6; % wrist position seen from tcp . TODO hardcode this
p_B_shoulder = [0;0;180;1]; % shoulder position seen from robot base . TODO hardcode this

for i = 1:ntasks
%     H_W_ti = tasks.getTask(i).getTransf();
    H_W_ti = tasks{i};
    p_B_wrist = H_B_W*H_W_ti*p_tcp_6; % compute hypothetical wrist position seen from robot base
    d = norm(p_B_wrist(1:3)-p_B_shoulder(1:3)); % compute distance between wrist and shoulder
    c(0+i) = - ( (d<(rmin+rmax)/2)*(d-rmin) + (d>=(rmin+rmax)/2)*(rmax-d) ); % c is negative when d€[rmin,rmax] and positive when out. The function is continuous, prolongable in (rmin+rmax)/2 and reaches its minimum there.
end

%% analytical IK + check limits
for i = 1:ntasks
%     q = computeIKfromShoulderWristTask(p_B_wrist,p_B_shoulder,R_0_7); % TODO . inspire from computeIKIiwa1

    %     Q = [[      q(1),  q(2),      q(3),  q(4),      q(5),  q(6),      q(7)];...
    %          [      q(1),  q(2), q(3) + pi, -q(4), q(5) + pi,  q(6),      q(7)];...
    %          [ q(1) + pi, -q(2),      q(3), -q(4), q(5) + pi,  q(6),      q(7)];...
    %          [ q(1) + pi, -q(2), q(3) + pi,  q(4),      q(5),  q(6),      q(7)];...
    %          [      q(1),  q(2),      q(3),  q(4), q(5) + pi, -q(6), q(7) - pi];...
    %          [      q(1),  q(2), q(3) + pi, -q(4),      q(5), -q(6), q(7) - pi];...
    %          [ q(1) + pi, -q(2),      q(3), -q(4),      q(5), -q(6), q(7) - pi];...
    %          [ q(1) + pi, -q(2), q(3) + pi,  q(4), q(5) + pi, -q(6), q(7) - pi]]; % compute all 8 config possibilities
    % Note : there are trivial possibilities for unreachable tasks, i.e. to
    % have at least one joint lying out of joint limits in each of the eight
    % configs. e.g. those that have q(2) > 120 or q(6) < -120.
    % Note2 : there are numerous possibilities for ureachable tasks...

    % Brut force approach : we will explore each of the 8 possibilities.
    % For each of them, we will return how far is each angle from the joint
    % limit
    
    
    % We are going to use "2008 - Analytical Inverse Kinematic Computation
    % for 7-DOF Redundant Manipulators With Joint Limits and Its
    % Application to Redundancy Resolution" to inquire about Beta_min and
    % Beta_max, for each of the eight configs
    
%     [q, D_Phi, indD_Phi] = computeIKIiwa2([0,0,0,0,0,0],tasks.getTask(i).getPose(),betas(i)); % D_Phi is negative when a configuration (of the 8 possibilities) is within joint limits, positive when not within joint limits and zero when exactely on joint limits. Behaviour is the same as with the distance
    [q, D_Phi, indD_Phi] = computeIKIiwa2([0,0,0,0,0,0],transformationToPoseEulerZYX(tasks{i}),betas(i));
    c(ntasks+i) = D_Phi*180/pi;
    
end

c(1:ntasks) = c(1:ntasks) + 20;
c(ntasks+1:2*ntasks) = c(ntasks+1:2*ntasks) + 5*pi/180;
ceq = [];



