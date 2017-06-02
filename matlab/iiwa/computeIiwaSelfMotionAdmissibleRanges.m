function [admSwiv,qq] = computeIiwaSelfMotionAdmissibleRanges(tcpPoseEuler, destPoseEuler, m_EE, transG_EE, showGraph, f, maxDeflection)
% use case : 
% qinit = (rand(1,7)-1/2).*[170,120,170,120,170,120,175]*2*pi/180;
% [swivel,FK] = getSwivelFromConfig(qinit);
% pose = transformation_to_peaZYX(FK{8});
% [admSwiv,qq] = computeIiwaSelfMotionAdmissibleRanges(zeros(1,6), pose, 8.5, [0,0,150], true, [0,0,-200,0,0,0]', {10*pi/180, 0.005});

% Analytical Iiwa 14 electric flange selfmotion computation
% function. Computes the self motion of the robot and the range of
% admissible swivel angles
% tcpPoseEuler    :
% destPoseEuler   :


% Algorithm based on "2008 - Analytical Inverse Kinematic Computation
% for 7-DOF Redundant Manipulators With Joint Limits and Its
% Application to Redundancy Resolution" to inquire about Beta_min and
% Beta_max, for each of the eight configs
    
% Q configs, pose within workspace, config within joint limits


ntestSwiv = 101;
swiv = linspace(-pi,pi,ntestSwiv);

if true % parameters settings
    Tmax = [320;320;176;176;110;40;40];
    Tmin = -Tmax;
    
    K=diag(1./[4e4  3.959e4    1.22e4     2.023e4    0.804e4     0.378e4    0.5e4]); % Nm/rad
    
end

if true % preliminaries
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
end

if true % check if target are reachable
% ----------------------------------------------
% Check reachability
% ----------------------------------------------

lUpperArm = 420;
lForeArm = 400;
L = norm(p_0_wrist-p_0_shoulder);


    if L > lUpperArm + lForeArm
        disp('unreachable target...');
        clear i;
        % TODO : do something about that
        qq = i;
        admSwiv = i;
        
        return;
    elseif L == lUpperArm + lForeArm
        disp ('arm fully extended : singularity ...');
        clear i;
        % TODO : do something about that
        qq = i;
        admSwiv = i;
        
        return;
    end
end

if true % shoulder elbow wrist, q4 computations
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

end

function ci = numOrDenShoulder(l,c,swiv)
    ci = As(l,c).*sin(swiv) + Bs(l,c).*cos(swiv) + Cs(l,c);
end

function ci = numOrDenWrist(l,c,swiv)
    ci = Aw(l,c).*sin(swiv) + Bw(l,c).*cos(swiv) + Cw(l,c);
end

function [r, d_jlim_min] = isWithinJointLimits(Q)
%     test avec Q = (1/2 - rand(7,8))*360
    dqij = repmat([170; 120; 170; 120; 170; 120; 175]*pi/180,[1 8]) - abs(Q);
    r = (sum(   (dqij  > 0)   ,1) == 7);
    
%     dqi_in = dqij(:,r == 1); % only have positive values within 
%     dqi_out = dqij(:,r == 0); % only have negative values within
%     
%     [d_jlim_min,indmin] = min(abs(dqij))
    
    if any(r)
        d_jlim_min = max(min(dqij(:,r)));
    else
        dqij(dqij>0) = -100000;
        d_jlim_min = min(max(dqij(:,~r))); % <=> min(max(dqij));
    end
end

function [J,Hs,Tg,Tf] = getJacobianAndTorques(q)
    
%     m_EE = 8.5; % kg
%     transG_EE = [0;0;125]; %mm
    f_trans = H_0_tcp(1:3,1:3)*f(1:3); % express f_trans in the robot base reference frame
    f_rot = H_0_tcp(1:3,1:3)*f(4:6); % express f_rot in the robot base reference frame

    [Tg,Hs] = getGravityTorquesIiwa(q, m_EE, transG_EE);
    [Tf,J] = getTorquesFromSpatialForceIiwa(q, tcpPoseEuler, [f_trans;f_rot]);
    
end



qq = zeros(7,8,ntestSwiv); % 7 x 8 x ntestSwiv
djlim = zeros(1,ntestSwiv);
lambda = zeros(1,ntestSwiv);
def_angle_z = zeros(1,ntestSwiv);
def_trans_xy = zeros(1,ntestSwiv);
admSwiv_jlim = zeros(1,ntestSwiv);
admSwiv_fcapa = zeros(1,ntestSwiv);
admSwiv_def_zangle = zeros(1,ntestSwiv);
admSwiv_def_xytrans = zeros(1,ntestSwiv);
for j = 1 : ntestSwiv
    
    if true % compute q for swiv(j)
        c2 = numOrDenShoulder(3,3,swiv(j));
        
        if (c2~=1 && c2~=-1)
            q1 = atan2(numOrDenShoulder(2,3,swiv(j)),numOrDenShoulder(1,3,swiv(j)));
            q3 = atan2(numOrDenShoulder(3,2,swiv(j)),-numOrDenShoulder(3,1,swiv(j)));
            if (sin(q1)~=0)
                q2 = atan2(numOrDenShoulder(2,3,swiv(j))./sin(q1),numOrDenShoulder(3,3,swiv(j)));
            else
                q2 = atan2(numOrDenShoulder(1,3,swiv(j))./cos(q1),numOrDenShoulder(3,3,swiv(j)));
            end
        else
            q1 = 0;
            q3 = atan2(numOrDenShoulder(2,1,swiv(j)),numOrDenShoulder(2,2,swiv(j)));
            q2 = acos(c2);
        end
        
        c6 = numOrDenWrist(2,3,swiv(j));
        if (c6~=1 && c6~=-1)
            q5 = atan2(-numOrDenWrist(3,3,swiv(j)),numOrDenWrist(1,3,swiv(j)));
            q7 = atan2(numOrDenWrist(2,2,swiv(j)),-numOrDenWrist(2,1,swiv(j)));
            
            if (cos(q5) ~= 0)
                q6 = atan2(numOrDenWrist(1,3,swiv(j))./cos(q5),numOrDenWrist(2,3,swiv(j)));
            else
                q6 = atan2(-numOrDenWrist(3,3,swiv(j))./sin(q5),numOrDenWrist(2,3,swiv(j)));
            end
            
        else
            q5 = 0;
            q7  = atan2(numOrDenWrist(3,1,swiv(j)),numOrDenWrist(3,2,swiv(j)));
            q6 = acos(c6);
        end
        
        q = [q1;q2;q3;q4;q5;q6;q7];
        qq(:,:,j) = setAnglesBetweenMinusPiAndPi([[ q(1),      q(1), pi + q(1), pi + q(1),      q(1),      q(1), pi + q(1), pi + q(1)];...
                                                  [ q(2),      q(2),     -q(2),     -q(2),      q(2),      q(2),     -q(2),     -q(2)];...
                                                  [ q(3), pi + q(3),      q(3), pi + q(3),      q(3), pi + q(3),      q(3), pi + q(3)];...
                                                  [ q(4),     -q(4),     -q(4),      q(4),      q(4),     -q(4),     -q(4),      q(4)];...
                                                  [ q(5), pi + q(5), pi + q(5),      q(5), pi + q(5),      q(5),      q(5), pi + q(5)];...
                                                  [ q(6),      q(6),      q(6),      q(6),     -q(6),     -q(6),     -q(6),     -q(6)];...
                                                  [ q(7),      q(7),      q(7),      q(7), q(7) - pi, q(7) - pi, q(7) - pi, q(7) - pi]]);
     
    end
    
    if true % check joint limits
        [r, djlim(j)] = isWithinJointLimits(qq(:,:,j));
        admSwiv_jlim(j) = any(r);
    end
    
    if true % compute Jacobian matrix, Tf and Tg
        [J,Hs,Tg,Tf] = getJacobianAndTorques(q); % only the first config is taken into account because force capa and deflections should be the same
    end
    
    if true % check force capability
        
        % compute force capacity for given config
        lambdas = (abs(Tg)<Tmax).* (abs((Tf>0).*(Tmax-Tg)./Tf + (Tf<0).*(Tmin-Tg)./Tf));

        lambda(j) = min( min(lambdas) , 999999 );

        admSwiv_fcapa(j) = (lambda(j)>1);
    end
    
    if true % check deflections
        % compute deformation for given config
        delta_q = K*(Tg+Tf);
        delta_xEE_0 = J*delta_q; % translational and rotational displacements of the tcp of the end effector expressed in the robot base reference frame 
        R_tcp_0 = (Hs{8}(1:3,1:3))'; % orientation of 0 seen from not-deflected tcp
        delta_xEE_tcp = [R_tcp_0*delta_xEE_0(1:3); R_tcp_0*delta_xEE_0(4:6)]; % translational and rotational displacements of the tcp of the end effector now expressed in the not-deflected tcp reference frame.
        
        % angle between the z axis of the not-deflected tcp and the
        % deflected tcp
        % 1) express delta_xEE_tcp as a rotation matrix R_tcp_tcp2
        angle_delta_xEE = norm(delta_xEE_tcp);
        skew_delta_xEE = skewSymMatrix(delta_xEE_tcp/angle_delta_xEE);
        R_tcp_tcp2 = eye(3) + skew_delta_xEE*sin(angle_delta_xEE) + skew_delta_xEE*(1-cos(angle_delta_xEE));
        
        % 2) extract the last column : this is the ztcp2 expressed in the not-deflected tcp reference frame (= ztcp2_tcp)
        ztcp2_tcp = R_tcp_tcp2(1:3,3);
        
        
        % 3) find the angle lying between [0;0;1] = ztcp_tcp and ztcp2_tcp
        def_angle_z(j) = angleBetweenUV([0;0;1], ztcp2_tcp); % rad
        admSwiv_def_zangle(j) = (maxDeflection{1} > def_angle_z(j));
        
        % x and y positionning errors
        def_trans_xy(j) = norm(delta_xEE_tcp(1:2)); % m
        admSwiv_def_xytrans(j) = (maxDeflection{2} > def_trans_xy(j));
    end
    
end

admSwiv = admSwiv_jlim & admSwiv_fcapa & admSwiv_def_zangle & admSwiv_def_xytrans; % 1 x 8 x ntestSwiv

if showGraph % display things only for the first computed config (all 7 others are not shown)
    figure(1);
    subplot(5,1,1)
    plot([-pi,pi],[0,0],'k',swiv,admSwiv_jlim, swiv, djlim)
    title('joint limits (rad)')
    subplot(5,1,2)
    plot([-pi,pi],[1,1],'k',swiv,admSwiv_fcapa, swiv, lambda)
    title('force capacity')
    subplot(5,1,3)
    plot([-pi,pi],[maxDeflection{1},maxDeflection{1}]*180/pi,'k',swiv,admSwiv_def_zangle, swiv, def_angle_z*180/pi)
    title('angle deflection of ztcp (°)')
    subplot(5,1,4)
    plot([-pi,pi],[maxDeflection{2},maxDeflection{2}]*1000,'k',[-pi,pi],-[maxDeflection{2},maxDeflection{2}]*1000,'k',swiv,admSwiv_def_xytrans, swiv, def_trans_xy*1000)
    title('tcp x-y positionning deflection (mm)')
    subplot(5,1,5)
    plot(swiv,admSwiv)
    title('admissible range');
end

end

