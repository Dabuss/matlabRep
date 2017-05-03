function Q = computeIKIiwa(tcpPoseEuler, destPoseEuler, swivel, varargin)
% Analytical Iiwa 14 pneumatic touch flange inverse geometry function
% tcpPoseEuler    :    
% destPoseEuler   :
% swivel          :  in rads
% varargin        :  check limits, change convention, receive former config

% Q configs, pose within workspace, config within joint limits 


% ----------------------------------------------
% Compute shoulder position
% ----------------------------------------------
p_0_shoulder = [0;0;360];

% ----------------------------------------------
% Compute wrist position
% ----------------------------------------------
H_0_tcp = poseEulerZYXToTransf(destPoseEuler);
H_7_tcp = poseEulerZYXToTransf(tcpPoseEuler);
H_6_7 = [1, 0,  0,  0   ;...
         0, 0, -1, -152 ;...
         0, 1,  0,  0   ;...
         0, 0,  0,  1   ]; % For iiwa 14 with pneumatic touch flange
H_0_6 = H_0_tcp*inverseTransformation(H_6_7*H_7_tcp);

p_0_wrist = H_0_6(1:3,4);

% ----------------------------------------------
% Check reachability
% ----------------------------------------------

lUpperArm = 420;
lForeArm = 400;
L = norm(p_0_wrist-p_0_shoulder);

if false
    if L > lUpperArm + lForeArm
        disp('unreachable target...'); 
        % TODO : do something about that
        return;
    elseif L == lUpperArm + lForeArm
        disp ('arm fully extended : singularity ...');
        % TODO : do something about that
        return;
    end
end
%-------------------------------------
% knowing the lUpperArm and lForeArm as well as swivelDeg, compute pElbow
%-------------------------------------

n = normalizeVector( p_0_wrist - p_0_shoulder ); % n direction goes from the shoulder to the wrist
% determine vector u, projection of uZ_world on the plane that carries
% the circle (C) on which the elbow has a degree of freedom. (Note:
% a singularity occurs if the shoulder-wrist segment is aligned with
% uz_world. In that case, u will be aligned with uY_world
u = vectorProjectedOnPlane([0,0,1]', n);
v = cross(n,u);
% compute p_elbow from u,n, the swivel angle and the lengths of the links

l1 = ( L*L - lForeArm*lForeArm + lUpperArm*lUpperArm )/( 2*L ); % length between shoulder and the center of the circle (C)
R = sqrt( lUpperArm*lUpperArm - l1*l1 ); % radius of the circle (C)

p_0_elbow = p_0_shoulder + n*l1 + R * ( u *cos(swivel) + v * sin(swivel) );


%-------------------------------------
% from p_0_elbow, p_0_shoulder & p_0_wrist, compute q1, q2, q3, q4, q5, q6, and q7
%-------------------------------------


% After computing the position of the elbow, we need to compute the
% positions of the 7 axis. The position of the elbow is already a function
% of the length of the forearm and the upperarm as well as the tcp frame.
% We can consider that the 3 first axes perform the relative orientation of
% the elbow in regard to the base of the robot, while the 3 latter perform
% the relative orientation of the TCP in regard to the one of the elbow.
% However, the elbow can have 2  different orientations wether it
% is a right arm elbow or a left arm elbow. Only the z4 direction is
% opposed. Moreover, the 3 first axes of the robot can match each of these
% orientations with two different sets of positions, leading up to 4
% different solutions bringing the upper-arm with the right position and
% orientation. After that, the 4th axis angle is forced by the angle
% between the shoulder-elbow segment and the elbow-wrist segment. Finally,
% the orientation and position of the 4th segment is completely known, as
% well as the one of the TCP. The 3 last axis positions have thus to
% create the relative orientation between these two frames. 2 sets of
% positions for these 3 axes are possible. If we don't consider joint limits,
% this leads to 8 different solutions for a given swivel angle. We'll
% relate these solutions to the status of the robot. the eight
% configurations will be gathered in a configuration matrix 'Q' with 8
% lines and 9 columns.

% Q = zeros(8,9);

% z0 = H_w_0(1:3,3);
% z7 = H_w_7(1:3,3);
SE = p_0_elbow - p_0_shoulder;
EW = p_0_wrist - p_0_elbow;
% p_0_circleCenter = p_0_shoulder + n*l1;
% CE = p_0_elbow - p_0_circleCenter;


% first step is to compute the four sets of angles of the 4 first axis

z3 = normalizeVector(SE);
y3 = -normalizeVector(cross(SE,EW));
x3 = cross(y3,z3);

R_0_3 = [x3, y3, z3];

% FGH = [X3 Y3 z3]. R_0_3 = R_0_1*R_1_2.
% We have FGH = R_0_1*R_1_3
% <=> R_1_0 * FGH = R_1_3
% equation (2,3) of this system leads to two solutions for q1 which
% settles the side on which the shoulder is.
% The elbow too can be on two sides, which changes the orientation of
% the 3rd and 4th base. This leads to two solutions for q2&q3&q4 for
% each solution of q1.
% equations (1,3) & (3,3) of this system lead to one solution for q2.
% equations (2,1) & (2,2) of this system lead to one solution for q3
% the angle between SE & EW helps computing q4

% q1 = atan2(hy, hx) and q1 = atan2(hy, hx) + pi
Q(1,1) = atan2(R_0_3(2,3), R_0_3(1,3)); % shoulder right
Q(2,1) = Q(1,1);                          % shoulder right
Q(3,1) = Q(1,1) + pi;                     % shoulder left
Q(4,1) = Q(1,1) + pi;                     % shoulder left

% for each of these solutions, q2 and q3 have one expression.
% q2 = atan2(hx*cos(q1)+hy*sin(q1), hz)
Q(1,2) = atan2(R_0_3(1,3)*cos(Q(1,1)) + R_0_3(2,3)*sin(Q(1,1)), R_0_3(3,3));
Q(2,2) = Q(1,2);
Q(3,2) = -Q(1,2);
Q(4,2) = -Q(1,2);

% q3 = atan2(fy*cos(q1)-fx*sin(q1), gy*cos(q1)-gx*sin(q1))
Q(1,3) = atan2(R_0_3(2,1)*cos(Q(1,1)) - ...
               R_0_3(1,1)*sin(Q(1,1)), ...
               R_0_3(2,2)*cos(Q(1,1)) - ...
               R_0_3(1,2)*sin(Q(1,1)));     % Shoulder right, elbow right
Q(2,3) = Q(1,3) + pi;                        % Shoulder right, elbow left
Q(3,3) = Q(1,3);                             % shoulder left, elbow left
Q(4,3) = Q(1,3) + pi;                        % shoulder left, elbow right

% q4 can be defined as the angle between x3 and x4.
%q4
y4 = normalizeVector(EW);
z4 = -y3;
x4 = cross(y4, z4);

if (isequal([0;0;0], x4-x3))
    Q(1,4) = 0;
    Q(2,4) = 0;
    Q(3,4) = 0;
    Q(4,4) = 0;
else
    %         Q(1,4) = real(asin(dot(cross(x3, x4),z4)));
    Q(1,4) = real(atan2(dot(cross(x3, x4),z4),dot(x3,x4))); % can be simpler ! (cosine law)
    Q(2,4) = -Q(1,4);
    Q(3,4) = -Q(1,4);
    Q(4,4) = Q(1,4);
end

Q(5:8,1:4) = Q(1:4,1:4); % duplicate the values of the first angles on the last lines

% second step is to compute the 2 sets of angles of the 3 last axes

% FGH = R_4_7 = R_4_0*R_0_7. R_4_7 = R_4_5*R_5_7.
% We have FGH = R_0_1*R_1_3
% <=> R_5_4 * FGH = R_5_7
%
% left member :
%  c5, 0, -s5    fx, gx, hx       fxc5?fzs5 , gxc5?gzs5, hxc5?hzs5
%(-s5, 0, -c5)* (fy, gy, hy) =  (?fzc5?fxs5 ,?gzc5?gxs5,?hzc5?hxs5)
%   0, 1, 0      fz, gz, hz           fy    ,      gy    ,    hy
%
% right member :
%           c7c6, -s7c6, s6
% R_5_7 = ( s7  ,   c7 ,   0 )
%          -c7s6, s7s6 , c6
%
% equation (2,3) of this system leads to two solutions for q5 which
% settles the side on which the wrist is.
% equations (1,3) & (3,3) of this system lead to one solution for q6.
% equations (2,1) & (2,2) of this system lead to one solution for q7.
    
% H_0_1 = [[ cos(Q(1,1)), -sin(Q(1,1)), 0,   0];...
%          [ sin(Q(1,1)),  cos(Q(1,1)), 0,   0];...
%          [           0,            0, 1, 360];...
%          [           0,            0, 0,   1]];  
% 
% H_1_2 = [[  cos(Q(1,2)), -sin(Q(1,2)), 0, 0];...
%          [            0,           0, 1, 0];...
%          [ -sin(Q(1,2)), -cos(Q(1,2)), 0, 0];...
%          [            0,           0, 0, 1]];
% 
% H_2_3 = [[ cos(Q(1,3)), -sin(Q(1,3)),  0,    0];...
%          [           0,           0, -1, -420];...
%          [ sin(Q(1,3)),  cos(Q(1,3)),  0,    0];...
%          [           0,           0,  0,    1]];
%      
% H_3_4 = [[ cos(Q(1,4)), -sin(Q(1,4)),  0,    0];...
%          [           0,           0, -1,    0];...
%          [ sin(Q(1,4)),  cos(Q(1,4)),  0,    0];...
%          [           0,           0,  0,    1]];   

c1 = cos(Q(1,1)); s1 = sin(Q(1,1));
c2 = cos(Q(1,2)); s2 = sin(Q(1,2));
c3 = cos(Q(1,3)); s3 = sin(Q(1,3));
c4 = cos(Q(1,4)); s4 = sin(Q(1,4));
 
H_0_4 = [[ c1*s2*s4 - c4*(s1*s3 - c1*c2*c3), s4*(s1*s3 - c1*c2*c3) + c1*c4*s2, c3*s1 + c1*c2*s3,    420*c1*s2];...
         [ c4*(c1*s3 + c2*c3*s1) + s1*s2*s4, c4*s1*s2 - s4*(c1*s3 + c2*c3*s1), c2*s1*s3 - c1*c3,    420*s1*s2];...
         [                 c2*s4 - c3*c4*s2,                 c2*c4 + c3*s2*s4,           -s2*s3, 420*c2 + 360];...
         [                                0,                                0,                0,            1]];

R_0_4 = H_0_4(1:3,1:3);

H_0_7= H_0_tcp*inverseTransformation(H_7_tcp);
R_0_7 = H_0_7(1:3,1:3);

R_4_7 = transpose(R_0_4)*R_0_7;


hz = R_4_7(3,3);
hy = R_4_7(2,3);
hx = R_4_7(1,3);
fz = R_4_7(3,1);
fx = R_4_7(1,1);
gz = R_4_7(3,2);
gx = R_4_7(1,2);

% q5 = atan2(-hz, hx) (+pi)
Q(1,5) = atan2(-hz, hx); % shoulder right, elbow right, wrist right
Q(2,5) = Q(1,5)+pi; % shoulder right, elbow left, wrist right
Q(3,5) = Q(2,5);           % shoulder left, elbow left, wrist right
Q(4,5) = Q(1,5);           % shoulder left, elbow right, wrist right
Q(5,5) = Q(2,5);      % shoulder right, elbow right, wrist left
Q(6,5) = Q(1,5);      % shoulder right, elbow left, wrist left
Q(7,5) = Q(1,5);      % shoulder left, elbow left, wrist left
Q(8,5) = Q(2,5);      % shoulder left, elbow right, wrist left

% q6 = atan2(hx*c5-hz*s5,hy)
Q(1,6) = atan2(hx*cos(Q(1,5))-hz*sin(Q(1,5)), hy);
Q(2,6) = Q(1,6);
Q(3,6) = Q(1,6);
Q(4,6) = Q(1,6);
Q(5,6) = -Q(1,6);
Q(6,6) = Q(5,6);
Q(7,6) = Q(5,6);
Q(8,6) = Q(5,6);

% q7 = atan2(-fz*c5-fx*s5,-gz*c5-gx*s5)
Q(1,7) = atan2(-fz*cos(Q(1,5))-fx*sin(Q(1,5)),-gz*cos(Q(1,5))-gx*sin(Q(1,5)));
Q(2,7) = Q(1,7);
Q(3,7) = Q(1,7);
Q(4,7) = Q(1,7);
Q(5,7) = Q(1,7) - pi;
Q(6,7) = Q(5,7);
Q(7,7) = Q(5,7);
Q(8,7) = Q(5,7);

Q = setAnglesBetweenMinusPiAndPi(Q);
    %if (checkJointLimits)
        a = zeros(8,1);
        for i=1:size(Q,1)
            conf = Q(i,1:7);
            a(i) = isIiwaConfigWithinLimits(conf*180/pi);
        end
        Q(:,8) = a;
    %end
end

