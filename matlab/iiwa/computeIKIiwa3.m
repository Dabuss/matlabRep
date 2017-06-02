function [Q, D_Phi, indD_Phi] = computeIKIiwa3(tcpPoseEuler, destPoseEuler, swivel, varargin)

% Analytical Iiwa 14 pneumatic touch flange inverse geometry function
% tcpPoseEuler    :
% destPoseEuler   :
% swivel          :  in rads
% varargin        :  check limits, change convention, receive former config

% Algorithm based on "2008 - Analytical Inverse Kinematic Computation
% for 7-DOF Redundant Manipulators With Joint Limits and Its
% Application to Redundancy Resolution" to inquire about Beta_min and
% Beta_max, for each of the eight configs
    
% Q configs, pose within workspace, config within joint limits

% ----------------------------------------------
% Compute shoulder position
% ----------------------------------------------
p_0_shoulder = [0;0;360];

% ----------------------------------------------
% Compute wrist position
% ----------------------------------------------
H_0_tcp = peaZYX_to_transformation(destPoseEuler);
H_7_tcp = peaZYX_to_transformation(tcpPoseEuler);
% H_6_7 = [1, 0,  0,  0   ;...
%          0, 0, -1, -152 ;...
%          0, 1,  0,  0   ;...
%          0, 0,  0,  1   ]; % For iiwa 14 with pneumatic touch flange
H_6_7 = [1, 0,  0,  0   ;...
    0, 0, -1, -126 ;...
    0, 1,  0,  0   ;...
    0, 0,  0,  1   ]; % For iiwa 14 with electric flange

H_0_7 = H_0_tcp*inverseTransformation(H_7_tcp);
H_0_6 = H_0_7*inverseTransformation(H_6_7);

p_0_wrist = H_0_6(1:3,4);

% ----------------------------------------------
% Check reachability
% ----------------------------------------------

lUpperArm = 420;
lForeArm = 400;
L = norm(p_0_wrist-p_0_shoulder);

if true
    if L > lUpperArm + lForeArm
        disp('unreachable target...');
        clear i;
        % TODO : do something about that
        Q = i;
        D_Phi = i;
        indD_Phi = i;
        
        return;
    elseif L == lUpperArm + lForeArm
        disp ('arm fully extended : singularity ...');
        clear i;
        % TODO : do something about that
        Q = i;
        D_Phi = i;
        indD_Phi = i;
        
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
r = sqrt( lUpperArm*lUpperArm - l1*l1 ); % radius of the circle (C)

p_0_elbow0 = p_0_shoulder + n*l1 + r *u ;


SE0 = p_0_elbow0 - p_0_shoulder;
E0W = p_0_wrist - p_0_elbow0;
SW = p_0_wrist - p_0_shoulder;
uSW = SW/L;

q4 = pi - acos((dot(SE0,SE0)+dot(E0W,E0W)-dot(SW,SW))/(2*(norm(SE0)*norm(E0W))));
c4 = cos(q4);
s4 = sin(q4);
% R_3_4 = [c4, -s4, 0; 0, 0, -1; s4, c4, 0];
R_4_3 = [c4, 0, s4; -s4, 0, c4; 0, -1, 0];




MuSW = skewSymMatrix( uSW );
% p_0_circleCenter = p_0_shoulder + n*l1;
% CE = p_0_elbow - p_0_circleCenter;


% first step is to compute the four sets of angles of the 4 first axis

z30 = SE0/lUpperArm; %normalizeVector(SE);
y30 = -normalizeVector(cross(z30,E0W));
x30 = cross(y30,z30);

R_0_30 = [x30, y30, z30];


As = MuSW*R_0_30;
Bs = -MuSW*As;
Cs = R_0_30-Bs;

R_0_7 = H_0_7(1:3,1:3);
Aw = R_4_3*As'*R_0_7;
Bw = R_4_3*Bs'*R_0_7;
Cw = R_4_3*Cs'*R_0_7;



ntestSwiv = 101;
span = 2*pi/ntestSwiv;
% swiv = linspace(-pi,pi,ntestSwiv);
if (swivel < 0)
    swiv = [fliplr(swivel:-span:-pi),(swivel+span):span:pi];
    ind_0 = length(swivel:-span:-pi);
else
    swiv = [fliplr((swivel-span):-span:-pi),swivel:span:pi];
    ind_0 = length((swivel-span):-span:-pi)+1;
end

    function ci = numOrDenShoulder(l,c,swiv)
        ci = As(l,c).*sin(swiv) + Bs(l,c).*cos(swiv) + Cs(l,c);
    end

    function ci = numOrDenWrist(l,c,swiv)
        ci = Aw(l,c).*sin(swiv) + Bw(l,c).*cos(swiv) + Cw(l,c);
    end


c2 = numOrDenShoulder(3,3,swiv);
boolVec = (c2~=1 & c2~=-1);

q1_s2diff0 = unwrap(atan2(numOrDenShoulder(2,3,swiv),numOrDenShoulder(1,3,swiv)));
q1s = zeros(1,length(swiv));
q1s(boolVec) = q1_s2diff0(boolVec);
% q1s = boolVec.*atan2(numOrDenShoulder(2,3,swiv),numOrDenShoulder(1,3,swiv)) + (~boolVec).*0;

q3_s2diff0 = unwrap(atan2(numOrDenShoulder(3,2,swiv),-numOrDenShoulder(3,1,swiv)));
q3_s2equal0 = unwrap(atan2(numOrDenShoulder(2,1,swiv),numOrDenShoulder(2,2,swiv)));
q3s = zeros(1,length(swiv));
q3s(boolVec) = q3_s2diff0(boolVec);
q3s(~boolVec) = q3_s2equal0(~boolVec);
% q3s = boolVec.*atan2(numOrDenShoulder(3,2,swiv),-numOrDenShoulder(3,1,swiv)) + (~boolVec).*atan2(numOrDenShoulder(2,1,swiv),numOrDenShoulder(2,2,swiv));

q2_s2diff0_and_s1diff0 = unwrap(atan2(numOrDenShoulder(2,3,swiv)./sin(q1s),numOrDenShoulder(3,3,swiv)));
q2_s2diff0_and_s1equal0 = unwrap(atan2(numOrDenShoulder(1,3,swiv)./cos(q1s),numOrDenShoulder(3,3,swiv)));
q2s = zeros(1,length(swiv));
q2s(boolVec & (sin(q1s)~=0)) = q2_s2diff0_and_s1diff0(boolVec & (sin(q1s)~=0));
q2s(boolVec & (sin(q1s)==0)) = q2_s2diff0_and_s1equal0(boolVec & (sin(q1s)==0));
% q2s = boolVec.*( (sin(q1s)~=0).*atan2(numOrDenShoulder(2,3,swiv)./sin(q1s),numOrDenShoulder(3,3,swiv)) + (sin(q1s)==0).*atan2(numOrDenShoulder(1,3,swiv)./cos(q1s),numOrDenShoulder(3,3,swiv))) + zeros(1,ntestSwiv); % ATTENTION : not able to equal pi if not boolVec, but should

q4s = q4*ones(1, length(swiv));

c6 = numOrDenWrist(2,3,swiv);
boolVec = (c6~=1 & c6~=-1);

q5_s6diff0 = unwrap(atan2(-numOrDenWrist(3,3,swiv),numOrDenWrist(1,3,swiv)));
q5s = zeros(1,length(swiv));
q5s(boolVec) = q5_s6diff0(boolVec);
% q5s = boolVec.*atan2(-numOrDenWrist(3,3,swiv),numOrDenWrist(1,3,swiv)) + (~boolVec).*0;

q7_s6diff0 = unwrap(atan2(numOrDenWrist(2,2,swiv),-numOrDenWrist(2,1,swiv)));
q7_s6equal0 = unwrap(atan2(numOrDenWrist(3,1,swiv),numOrDenWrist(3,2,swiv)));
q7s = zeros(1,length(swiv));
q7s(boolVec) = q7_s6diff0(boolVec);
q7s(~boolVec) = q7_s6equal0(~boolVec);
% q7s = boolVec.*atan2(numOrDenWrist(2,2,swiv),-numOrDenWrist(2,1,swiv)) + (~boolVec).*atan2(numOrDenWrist(3,1,swiv),numOrDenWrist(3,2,swiv));

q6_s2diff6_and_c5diff0 = unwrap(atan2(numOrDenWrist(1,3,swiv)./cos(q5s),numOrDenWrist(2,3,swiv)));
q6_s6diff0_and_c5equal0 = unwrap(atan2(-numOrDenWrist(3,3,swiv)./sin(q5s),numOrDenWrist(2,3,swiv)));
q6s = zeros(1,length(swiv));
q6s(boolVec & (cos(q5s)~=0)) = q6_s2diff6_and_c5diff0(boolVec & (cos(q5s)~=0));
q6s(boolVec & (cos(q5s)==0)) = q6_s6diff0_and_c5equal0(boolVec & (cos(q5s)==0));
% q6s = boolVec.*( (cos(q5s)~=0).*atan2(numOrDenWrist(1,3,swiv)./cos(q5s),numOrDenWrist(2,3,swiv)) + (cos(q5s)==0).*atan2(-numOrDenWrist(3,3,swiv)./sin(q5s),numOrDenWrist(2,3,swiv)) ) + zeros(1,ntestSwiv);


%     Q = [[      q(1),  q(2),      q(3),  q(4),      q(5),  q(6),      q(7)];...
%          [      q(1),  q(2), q(3) + pi, -q(4), q(5) + pi,  q(6),      q(7)];...
%          [ q(1) + pi, -q(2),      q(3), -q(4), q(5) + pi,  q(6),      q(7)];...
%          [ q(1) + pi, -q(2), q(3) + pi,  q(4),      q(5),  q(6),      q(7)];...

%          [      q(1),  q(2),      q(3),  q(4), q(5) + pi, -q(6), q(7) - pi];...
%          [      q(1),  q(2), q(3) + pi, -q(4),      q(5), -q(6), q(7) - pi];...
%          [ q(1) + pi, -q(2),      q(3), -q(4),      q(5), -q(6), q(7) - pi];...
%          [ q(1) + pi, -q(2), q(3) + pi,  q(4), q(5) + pi, -q(6), q(7) - pi]]; % compute all 8 config possibilities
qq = zeros(7,length(swiv),8); %lines are the 7 joints, columns are for swivel, depth is for the 8 possible configurations
qq(:,:,1) = [q1s;q2s;q3s;q4s;q5s;q6s;q7s];
qq(:,:,2) = [q1s;q2s;q3s+pi;-q4s;q5s+pi;q6s;q7s];
qq(:,:,3) = [q1s+pi;-q2s;q3s;-q4s;q5s+pi;q6s;q7s];
qq(:,:,4) = [q1s+pi;-q2s;q3s+pi;q4s;q5s;q6s;q7s];
qq(:,:,5) = [q1s;q2s;q3s;q4s;q5s+pi;-q6s;q7s-pi];
qq(:,:,6) = [q1s;q2s;q3s+pi;-q4s;q5s;-q6s;q7s-pi];
qq(:,:,7) = [q1s+pi;-q2s;q3s;-q4s;q5s;-q6s;q7s-pi];
qq(:,:,8) = [q1s;q2s;q3s;q4s;q5s+pi;-q6s;q7s-pi];



% qq organization :
%             q1(-pi) q1(-pi+h) q1(-pi+2h) % q1 pour la 3 eme config
%       q1(-pi) q1(-pi+h) q1(-pi+2h)  % q1 pour la 2 eme config
% q1(-pi) q1(-pi+h) q1(-pi+2h) % q1 pour la 1ere config 
% q2(-pi) q2(-pi+h) q2(-pi+2h) % q2 pour la 1ere config 
% q3(-pi) q3(-pi+h) q3(-pi+2h) % q3 pour la 1ere config
% q4(-pi) q4(-pi+h) q4(-pi+2h) % q4 pour la 1ere config
% q5(-pi) q5(-pi+h) q5(-pi+2h) % q5 pour la 1ere config
% q6(-pi) q6(-pi+h) q6(-pi+2h) % q6 pour la 1ere config
% q7(-pi) q7(-pi+h) q7(-pi+2h) % q7 pour la 1ere config


% create joints limits lines
piinf = -pi-eps;
pisup = pi+eps;
xlimitsLines = [pisup, piinf, piinf, pisup, pisup, piinf, piinf, pisup, pisup, piinf, piinf, pisup];

function r = iswithin(x, low,hi)
    % returns logical true if low < x < hi

    r = x>=low & x<=hi;
end

admissibleRanges = cell(7,1,8);
ranges = cell(1,1,8);
D_Phis = zeros(8,1);

for ind_config = 1:8
    
    isSwivelWithinJointLimits = true;
    for j = 1:7
        switch j
            case 1
                jlim = 170*pi/180;
            case 2
                jlim = 120*pi/180;
            case 3
                jlim = 170*pi/180;
            case 4
                jlim = 120*pi/180;
            case 5
                jlim = 170*pi/180;
            case 6
                jlim = 120*pi/180;
            case 7
                jlim = 175*pi/180;
        end
            
        
        jlimitsLine = [xlimitsLines;...
                -jlim-2*pi, -jlim-2*pi, +jlim-2*pi, +jlim-2*pi, -jlim, -jlim, jlim, jlim, -jlim+2*pi, -jlim+2*pi, +jlim+2*pi, +jlim+2*pi];
            
        
        

        
        

        % compute admissible ranges for one particular joint for one particular config
        inters = InterX(jlimitsLine, [swiv;qq(j,:,ind_config)]);
        intersSwivel = [-pi,inters(1,:),pi];
        l = length(intersSwivel);
        isEven = (mod(l,2) == 0);
        
%         figure(1)
%         plot(jlimitsLine(1,:), jlimitsLine(2,:));
%         hold on;
%         plot(swiv,qq(j,:,ind_config));
%         plot(inters(1,:),inters(2,:),'o')
%         plot([swiv(ind_0),swiv(ind_0)],[-3*pi,3*pi])
%         hold off;
        
        
        isSwivelWithinJointLimits = isSwivelWithinJointLimits && ...
                                    iswithin(setAnglesBetweenMinusPiAndPi(qq(j,ind_0,ind_config)),-jlim, +jlim);
        
        if iswithin(setAnglesBetweenMinusPiAndPi(qq(j,1,ind_config)),-jlim, +jlim) % qq(j,1,i) ( = qqj(-pi) ) is within limits
            if isEven
                admissibleRanges{j,1,ind_config}(:) = intersSwivel;
            else
                admissibleRanges{j,1,ind_config}(:) = intersSwivel(1:end-1);
            end
        else % qq(j,1,i) ( = qqj(-pi) ) isn't within limits
            if isEven
                admissibleRanges{j,1,ind_config}(:) = intersSwivel(2:end-1);
            else
                admissibleRanges{j,1,ind_config}(:) = intersSwivel(2:end);
            end
        end
        
        % compute admissible ranges for all 7 joints
        if j == 1
            ranges{ind_config} = admissibleRanges{j,1,ind_config};
        else
            ranges{ind_config} = range_intersection(ranges{ind_config},admissibleRanges{j,1,ind_config});
        end
        
        
        
    end
    % clean ranges{i} so that there is no zero length spans (may return a
    % span with twice the same number, which is not to be taken into
    % account
    twoRowsRanges = zeros(2,length(ranges{ind_config})/2);
    twoRowsRanges(:) = ranges{ind_config};
    twoRowsRanges = twoRowsRanges(:, diff(twoRowsRanges,1) ~= 0);
    
    % merge spans that can be merged together (Roger Stafford answer at http://fr.mathworks.com/matlabcentral/newsreader/view_thread/171594)
    A = twoRowsRanges';
    n = size(A,1);
    [t,p] = sort(A(:));
    z = cumsum(accumarray((1:2*n)',2*(p<=n)-1));
    z1 = [0;z(1:end-1)];
    twoRowsRanges2 = [t(z1==0 & z>0),t(z1>0 & z==0)];
    
    ranges{ind_config} = zeros(1,size(twoRowsRanges2,1)*size(twoRowsRanges2,2));
    ranges{ind_config}(:) = twoRowsRanges2;
    
    % store D_phis(i), the distance to being within joint limits for config i
    if ~isempty(ranges{ind_config})
        D_Phis(ind_config) = min( abs(ranges{ind_config} - swivel) )*( -(isSwivelWithinJointLimits) + (~isSwivelWithinJointLimits) ); % is negative if within and positive if out of joints limits
    else
        D_Phis(ind_config) = pi*( -(isSwivelWithinJointLimits) + (~isSwivelWithinJointLimits) );
    end
    
    
end


[D_Phi, indD_Phi] = min(D_Phis); 


% q needs to be set within -pi and pi
Q = permute(setAnglesBetweenMinusPiAndPi(qq(:,ind_0,:)),[1 3 2])';


end