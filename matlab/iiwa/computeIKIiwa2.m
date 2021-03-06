function [Q, D_Phi, indD_Phi] = computeIKIiwa2(tcpPoseEuler, destPoseEuler, swivel, varargin)

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
% v = cross(n,u);
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

q1_s2diff0 = atan2(numOrDenShoulder(2,3,swiv),numOrDenShoulder(1,3,swiv));
q1s = zeros(1,length(swiv));
q1s(boolVec) = q1_s2diff0(boolVec);
% q1s = boolVec.*atan2(numOrDenShoulder(2,3,swiv),numOrDenShoulder(1,3,swiv)) + (~boolVec).*0;

q3_s2diff0 = atan2(numOrDenShoulder(3,2,swiv),-numOrDenShoulder(3,1,swiv));
q3_s2equal0 = atan2(numOrDenShoulder(2,1,swiv),numOrDenShoulder(2,2,swiv));
q3s = zeros(1,length(swiv));
q3s(boolVec) = q3_s2diff0(boolVec);
q3s(~boolVec) = q3_s2equal0(~boolVec);
% q3s = boolVec.*atan2(numOrDenShoulder(3,2,swiv),-numOrDenShoulder(3,1,swiv)) + (~boolVec).*atan2(numOrDenShoulder(2,1,swiv),numOrDenShoulder(2,2,swiv));

q2_s2diff0_and_s1diff0 = atan2(numOrDenShoulder(2,3,swiv)./sin(q1s),numOrDenShoulder(3,3,swiv));
q2_s2diff0_and_s1equal0 = atan2(numOrDenShoulder(1,3,swiv)./cos(q1s),numOrDenShoulder(3,3,swiv));
q2s = zeros(1,length(swiv));
q2s(boolVec & (sin(q1s)~=0)) = q2_s2diff0_and_s1diff0(boolVec & (sin(q1s)~=0));
q2s(boolVec & (sin(q1s)==0)) = q2_s2diff0_and_s1equal0(boolVec & (sin(q1s)==0));
% q2s = boolVec.*( (sin(q1s)~=0).*atan2(numOrDenShoulder(2,3,swiv)./sin(q1s),numOrDenShoulder(3,3,swiv)) + (sin(q1s)==0).*atan2(numOrDenShoulder(1,3,swiv)./cos(q1s),numOrDenShoulder(3,3,swiv))) + zeros(1,ntestSwiv); % ATTENTION : not able to equal pi if not boolVec, but should

q4s = q4*ones(1, length(swiv));

c6 = numOrDenWrist(2,3,swiv);
boolVec = (c6~=1 & c6~=-1);

q5_s6diff0 = atan2(-numOrDenWrist(3,3,swiv),numOrDenWrist(1,3,swiv));
q5s = zeros(1,length(swiv));
q5s(boolVec) = q5_s6diff0(boolVec);
% q5s = boolVec.*atan2(-numOrDenWrist(3,3,swiv),numOrDenWrist(1,3,swiv)) + (~boolVec).*0;

q7_s6diff0 = atan2(numOrDenWrist(2,2,swiv),-numOrDenWrist(2,1,swiv));
q7_s6equal0 = atan2(numOrDenWrist(3,1,swiv),numOrDenWrist(3,2,swiv));
q7s = zeros(1,length(swiv));
q7s(boolVec) = q7_s6diff0(boolVec);
q7s(~boolVec) = q7_s6equal0(~boolVec);
% q7s = boolVec.*atan2(numOrDenWrist(2,2,swiv),-numOrDenWrist(2,1,swiv)) + (~boolVec).*atan2(numOrDenWrist(3,1,swiv),numOrDenWrist(3,2,swiv));

q6_s2diff6_and_c5diff0 = atan2(numOrDenWrist(1,3,swiv)./cos(q5s),numOrDenWrist(2,3,swiv));
q6_s6diff0_and_c5equal0 = atan2(-numOrDenWrist(3,3,swiv)./sin(q5s),numOrDenWrist(2,3,swiv));
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
qq(:,:,2) = setAnglesBetweenMinusPiAndPi([q1s;q2s;q3s+pi;-q4s;q5s+pi;q6s;q7s]);
qq(:,:,3) = setAnglesBetweenMinusPiAndPi([q1s+pi;-q2s;q3s;-q4s;q5s+pi;q6s;q7s]);
qq(:,:,4) = setAnglesBetweenMinusPiAndPi([q1s+pi;-q2s;q3s+pi;q4s;q5s;q6s;q7s]);
qq(:,:,5) = setAnglesBetweenMinusPiAndPi([q1s;q2s;q3s;q4s;q5s+pi;-q6s;q7s-pi]);
qq(:,:,6) = setAnglesBetweenMinusPiAndPi([q1s;q2s;q3s+pi;-q4s;q5s;-q6s;q7s-pi]);
qq(:,:,7) = setAnglesBetweenMinusPiAndPi([q1s+pi;-q2s;q3s;-q4s;q5s;-q6s;q7s-pi]);
qq(:,:,8) = setAnglesBetweenMinusPiAndPi([q1s;q2s;q3s;q4s;q5s+pi;-q6s;q7s-pi]);



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




% jointLimits = [170; 120; 170; 120; 170; 120; 175];
b_q1withinLimits = (qq(1,:,:) < 170*pi/180 & qq(1,:,:) > -170*pi/180); %flat matrix gathering if q1 is within its limits for each config, for the different swivel angles
b_q2withinLimits = (qq(2,:,:) < 120*pi/180 & qq(2,:,:) > -120*pi/180);
b_q3withinLimits = (qq(3,:,:) < 170*pi/180 & qq(3,:,:) > -170*pi/180);
b_q4withinLimits = (qq(4,:,:) < 120*pi/180 & qq(4,:,:) > -120*pi/180);
b_q5withinLimits = (qq(5,:,:) < 170*pi/180 & qq(5,:,:) > -170*pi/180);
b_q6withinLimits = (qq(6,:,:) < 120*pi/180 & qq(6,:,:) > -120*pi/180);
b_q7withinLimits = (qq(7,:,:) < 175*pi/180 & qq(7,:,:) > -175*pi/180);



b_qwithinLimits = b_q1withinLimits...
    & b_q2withinLimits...
    & b_q3withinLimits...
    & b_q4withinLimits...
    & b_q5withinLimits...
    & b_q6withinLimits...
    & b_q7withinLimits;


% compute the distance from feasible or unfeasible swivel angle
D_Phis = zeros(1,8);
for j = 1:8
    
    if (~any(b_qwithinLimits(1,:,j)-1)) %if config i is within joint limits no matter what swiv
        D_Phis(j) = -pi;
    elseif (~any(b_qwithinLimits(1,:,j))) %if config i is out joint limits no matter what swiv
        D_Phis(j) = pi;
    else
        [d_phis,indmin] = min(abs(swiv( b_qwithinLimits(1,:,j) ~= b_qwithinLimits(1,ind_0,j)) - swivel)); %computed distance (i.e. constraint) is negative
%         swiv(indmin)
%         swivel
%         swiv(indmin) - swivel
        if b_qwithinLimits(1,ind_0,j) %if config is within limits for the given swivel
            [D_Phis(j)] = -d_phis; %computed distance (i.e. constraint) is negative
        else
            [D_Phis(j)] = d_phis; %computed distance is positive
        end
    end
    
%     figure(2)
%     n=1;
%     step = 6;
%     offset = -n*step; n=n+1;
%     plot(swiv,qq(1,:,j)+offset,'k',[-pi,pi],offset+[170*pi/180,170*pi/180],'r',[-pi,pi],offset+[-170*pi/180,-170*pi/180],'r')
%     hold on
%     offset = -n*step; n=n+1;
%     plot(swiv,qq(2,:,j)+offset,'k',[-pi,pi],offset+[120*pi/180,120*pi/180],'r',[-pi,pi],offset+[-120*pi/180,-120*pi/180],'r')
%     offset = -n*step; n=n+1;
%     plot(swiv,qq(3,:,j)+offset,'k',[-pi,pi],offset+[170*pi/180,170*pi/180],'r',[-pi,pi],offset+[-170*pi/180,-170*pi/180],'r')
%     offset = -n*step; n=n+1;
%     plot(swiv,qq(4,:,j)+offset,'k',[-pi,pi],offset+[120*pi/180,120*pi/180],'r',[-pi,pi],offset+[-120*pi/180,-120*pi/180],'r')
%     offset = -n*step; n=n+1;
%     plot(swiv,qq(5,:,j)+offset,'k',[-pi,pi],offset+[170*pi/180,170*pi/180],'r',[-pi,pi],offset+[-170*pi/180,-170*pi/180],'r')
%     offset = -n*step; n=n+1;
%     plot(swiv,qq(6,:,j)+offset,'k',[-pi,pi],offset+[120*pi/180,120*pi/180],'r',[-pi,pi],offset+[-120*pi/180,-120*pi/180],'r')
%     offset = -n*step; n=n+1;
%     plot(swiv,qq(7,:,j)+offset,'k',[-pi,pi],offset+[175*pi/180,175*pi/180],'r',[-pi,pi],offset+[-175*pi/180,-175*pi/180],'r')
%     plot([swivel,swivel],[0,-50],'b')
%     plot([swiv(indmin),swiv(indmin)],[0,-50],'g')
%     
%     chgSwiv = swiv((diff(b_qwithinLimits(1,:,j)) ~= 0 ));
%     for ind = 1 : length(chgSwiv)
%         plot([chgSwiv(ind), chgSwiv(ind)], [0,-50],'k')
%         plot(swiv(b_qwithinLimits(1,:,j)),-25 + zeros(1,length(swiv(b_qwithinLimits(1,:,j)))),'xg',swivel,0,'o');
%     end
%     
%     
%     title(num2str(D_Phis(j)));
% %     [x,y] = ginput(2);
% %     title([num2str(D_Phis(j)),' ', num2str(norm([x(1),y(1)]-[x(2),y(2)]))]);
%     hold off
%     bob=1;
end

[D_Phi, indD_Phi] = min(D_Phis);

% figure(1);
% title(num2str(D_Phi))
% subplot(8,1,1);
% plot(swiv,qq(1,:,indD_Phi),'k',[-pi,pi],[170*pi/180,170*pi/180],'r',[-pi,pi],[-170*pi/180,-170*pi/180],'r')
% subplot(8,1,2);
% plot(swiv,qq(2,:,indD_Phi),'k',[-pi,pi],[120*pi/180,120*pi/180],'r',[-pi,pi],[-120*pi/180,-120*pi/180],'r')
% subplot(8,1,3);
% plot(swiv,qq(3,:,indD_Phi),'k',[-pi,pi],[170*pi/180,170*pi/180],'r',[-pi,pi],[-170*pi/180,-170*pi/180],'r')
% subplot(8,1,4);
% plot(swiv,qq(4,:,indD_Phi),'k',[-pi,pi],[120*pi/180,120*pi/180],'r',[-pi,pi],[-120*pi/180,-120*pi/180],'r')
% subplot(8,1,5);
% plot(swiv,qq(5,:,indD_Phi),'k',[-pi,pi],[170*pi/180,170*pi/180],'r',[-pi,pi],[-170*pi/180,-170*pi/180],'r')
% subplot(8,1,6);
% plot(swiv,qq(6,:,indD_Phi),'k',[-pi,pi],[120*pi/180,120*pi/180],'r',[-pi,pi],[-120*pi/180,-120*pi/180],'r')
% subplot(8,1,7);
% plot(swiv,qq(7,:,indD_Phi),'k',[-pi,pi],[175*pi/180,175*pi/180],'r',[-pi,pi],[-175*pi/180,-175*pi/180],'r')
% subplot(8,1,8);
% plot(swiv(b_qwithinLimits(1,:,1)),zeros(1,length(swiv(b_qwithinLimits(1,:,1)))),'xg',swivel,0,'o');
% axis([-4,4,-1,1])

% figure(2)
% n=1;
% step = 6;
% offset = -n*step; n=n+1;
% plot(swiv,qq(1,:,indD_Phi)+offset,'k',[-pi,pi],offset+[170*pi/180,170*pi/180],'r',[-pi,pi],offset+[-170*pi/180,-170*pi/180],'r')
% hold on
% offset = -n*step; n=n+1;
% plot(swiv,qq(2,:,indD_Phi)+offset,'k',[-pi,pi],offset+[120*pi/180,120*pi/180],'r',[-pi,pi],offset+[-120*pi/180,-120*pi/180],'r')
% offset = -n*step; n=n+1;
% plot(swiv,qq(3,:,indD_Phi)+offset,'k',[-pi,pi],offset+[170*pi/180,170*pi/180],'r',[-pi,pi],offset+[-170*pi/180,-170*pi/180],'r')
% offset = -n*step; n=n+1;
% plot(swiv,qq(4,:,indD_Phi)+offset,'k',[-pi,pi],offset+[120*pi/180,120*pi/180],'r',[-pi,pi],offset+[-120*pi/180,-120*pi/180],'r')
% offset = -n*step; n=n+1;
% plot(swiv,qq(5,:,indD_Phi)+offset,'k',[-pi,pi],offset+[170*pi/180,170*pi/180],'r',[-pi,pi],offset+[-170*pi/180,-170*pi/180],'r')
% offset = -n*step; n=n+1;
% plot(swiv,qq(6,:,indD_Phi)+offset,'k',[-pi,pi],offset+[120*pi/180,120*pi/180],'r',[-pi,pi],offset+[-120*pi/180,-120*pi/180],'r')
% offset = -n*step; n=n+1;
% plot(swiv,qq(7,:,indD_Phi)+offset,'k',[-pi,pi],offset+[175*pi/180,175*pi/180],'r',[-pi,pi],offset+[-175*pi/180,-175*pi/180],'r')
% plot([swivel,swivel],[0,-50],'b')
% % plot(swiv(b_qwithinLimits(1,:,1)),offset + zeros(1,length(swiv(b_qwithinLimits(1,:,1)))),'xg',swivel,0,'o');
% 
% hold off



Q = permute(qq(:,ind_0,:),[1 3 2])';


end