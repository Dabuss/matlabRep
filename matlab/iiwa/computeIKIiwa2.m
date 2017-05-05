function [Q, D_Phi, indD_Phi] = computeIKIiwa2(tcpPoseEuler, destPoseEuler, swivel, varargin)

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
r = sqrt( lUpperArm*lUpperArm - l1*l1 ); % radius of the circle (C)

% p_0_elbow = p_0_shoulder + n*l1 + r * ( u *cos(swivel) + v * sin(swivel) );
p_0_elbow0 = p_0_shoulder + n*l1 + r *u ;

%-------------------------------------
% from p_0_elbow, p_0_shoulder & p_0_wrist, compute q1, q2, q3, q4, q5, q6, and q7
%-------------------------------------

% compute q4
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
    ind_0 = length(-pi:span:swivel);
else
    swiv = [fliplr(-pi:span:(swivel-span)),swivel:span:pi];
    ind_0 = length(-pi:span:(swivel-span))+1;
end

%     function ti = tanType(l1,c1,l2,c2)
%         ti = (As(l1,c1).*sin(swiv) + Bs(l1,c1).*cos(swiv) + Cs(l1,c1))./...
%              (As(l2,c2).*sin(swiv) + Bs(l2,c2).*cos(swiv) + Cs(l2,c2));
%     end

    function ci = numOrDenShoulder(l,c,swiv)
        ci = As(l,c).*sin(swiv) + Bs(l,c).*cos(swiv) + Cs(l,c);
    end

    function ci = numOrDenWrist(l,c,swiv)
        ci = Aw(l,c).*sin(swiv) + Bw(l,c).*cos(swiv) + Cw(l,c);
    end
    
%     function qis = atan2Types(l1,c1,l2,c2)
%         C1 = abs(c1);
%         L1 = abs(l1);
%         C2 = abs(c2);
%         L2 = abs(l2);
%         qis = atan2(sign(l1)*(As(L1,C1).*sin(swiv) + Bs(L1,C1).*cos(swiv) + Cs(L1,C1)), ...
%                     sign(l2)*(As(L2,C2).*sin(swiv) + Bs(L2,C2).*cos(swiv) + Cs(L2,C2)));
%     end


% q1s = atan2Types(2,3,1,3); % works if sin(q2) >< 0 !!!
% q3s = atan2Types(3,2,-3,-1); 
% 
% t1 = tanType(2,3,1,3);
% c2 = cosType(3,3);
% t3 = -tanType(3,2,3,1);


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




% jointLimits = [170; 120; 170; 120; 170; 120; 175];
b_q1withinLimits = (qq(1,:,:) < 170*pi/180 & qq(1,:,:) > -170*pi/180); %flat matrix gathering if q1 is within its limits for each config, for the different swivel angles
b_q2withinLimits = (qq(2,:,:) < 120*pi/180 & qq(2,:,:) > -120*pi/180);
b_q3withinLimits = (qq(3,:,:) < 170*pi/180 & qq(3,:,:) > -170*pi/180);
b_q4withinLimits = (qq(4,:,:) < 120*pi/180 & qq(4,:,:) > -120*pi/180);
b_q5withinLimits = (qq(5,:,:) < 170*pi/180 & qq(5,:,:) > -170*pi/180);
b_q6withinLimits = (qq(6,:,:) < 120*pi/180 & qq(6,:,:) > -120*pi/180);
b_q7withinLimits = (qq(7,:,:) < 175*pi/180 & qq(7,:,:) > -175*pi/180);
% etc...


b_qwithinLimits = b_q1withinLimits...
                  & b_q2withinLimits...
                  & b_q3withinLimits...
                  & b_q4withinLimits...
                  & b_q5withinLimits...
                  & b_q6withinLimits...
                  & b_q7withinLimits;
              
% ind_0 = floor(swivel*ntestSwiv/(2*pi))+ floor(ntestSwiv/2) + 1;

% ind_0 = b_qwithinLimits( floor(swivel*ntestSwiv/(2*pi))+ floor(ntestSwiv/2) + 1); % that is where would swivel place within swiv.

% compute the distance from feasible or unfeasible swivel angle
D_Phis = zeros(1,8);
for i = 1:8
    if (~any(b_qwithinLimits(1,:,i)-1)) %if config i is within joint limits no matter what swiv
        D_Phis(i) = -pi;
    elseif (~any(b_qwithinLimits(1,:,i))) %if config i is out joint limits no matter what swiv
        D_Phis(i) = pi;
    else
        if b_qwithinLimits(1,ind_0,i) %if config is within limits for the given swivel
            D_Phis(i) = -min(abs(swiv( b_qwithinLimits(1,:,i) ~= b_qwithinLimits(1,ind_0,i)) - swivel)); %computed distance (i.e. constraint) is negative
        else
            D_Phis(i) = min(abs(swiv( b_qwithinLimits(1,:,i) ~= b_qwithinLimits(1,ind_0,i)) - swivel)); %computed distance is positive
        end
    end
end

% D_Phi_possibility_1 = pi*(~any(b_qwithinLimits(1,:,1)-1)) - pi*(~any(b_qwithinLimits(1,:,1))) + min(abs(swiv( b_qwithinLimits(1,:,1) ~= b_qwithinLimits(1,ind_0,1)) - swivel));
% D_Phi_possibility_2 = pi*(~any(b_qwithinLimits(1,:,2)-1)) - pi*(~any(b_qwithinLimits(1,:,2))) + min(abs(swiv( b_qwithinLimits(1,:,2) ~= b_qwithinLimits(1,ind_0,2)) - swivel));
% D_Phi_possibility_3 = pi*(~any(b_qwithinLimits(1,:,3)-1)) - pi*(~any(b_qwithinLimits(1,:,3))) + min(abs(swiv( b_qwithinLimits(1,:,3) ~= b_qwithinLimits(1,ind_0,3)) - swivel));
% D_Phi_possibility_4 = pi*(~any(b_qwithinLimits(1,:,4)-1)) - pi*(~any(b_qwithinLimits(1,:,4))) + min(abs(swiv( b_qwithinLimits(1,:,4) ~= b_qwithinLimits(1,ind_0,4)) - swivel));
% D_Phi_possibility_5 = pi*(~any(b_qwithinLimits(1,:,5)-1)) - pi*(~any(b_qwithinLimits(1,:,5))) + min(abs(swiv( b_qwithinLimits(1,:,5) ~= b_qwithinLimits(1,ind_0,5)) - swivel));
% D_Phi_possibility_6 = pi*(~any(b_qwithinLimits(1,:,6)-1)) - pi*(~any(b_qwithinLimits(1,:,6))) + min(abs(swiv( b_qwithinLimits(1,:,6) ~= b_qwithinLimits(1,ind_0,6)) - swivel));
% D_Phi_possibility_7 = pi*(~any(b_qwithinLimits(1,:,7)-1)) - pi*(~any(b_qwithinLimits(1,:,7))) + min(abs(swiv( b_qwithinLimits(1,:,7) ~= b_qwithinLimits(1,ind_0,7)) - swivel));
% D_Phi_possibility_8 = pi*(~any(b_qwithinLimits(1,:,8)-1)) - pi*(~any(b_qwithinLimits(1,:,8))) + min(abs(swiv( b_qwithinLimits(1,:,8) ~= b_qwithinLimits(1,ind_0,8)) - swivel));

[D_Phi, indD_Phi] = min(D_Phis);

% figure(1);
% subplot(8,1,1);
% plot(swiv,q1s,'k',[-pi,pi],[170*pi/180,170*pi/180],'r',[-pi,pi],[-170*pi/180,-170*pi/180],'r')
% subplot(8,1,2);
% plot(swiv,q2s,'k',[-pi,pi],[120*pi/180,120*pi/180],'r',[-pi,pi],[-120*pi/180,-120*pi/180],'r')
% subplot(8,1,3);
% plot(swiv,q3s,'k',[-pi,pi],[170*pi/180,170*pi/180],'r',[-pi,pi],[-170*pi/180,-170*pi/180],'r')
% subplot(8,1,4);
% plot(swiv,q4s,'k',[-pi,pi],[120*pi/180,120*pi/180],'r',[-pi,pi],[-120*pi/180,-120*pi/180],'r')
% subplot(8,1,5);
% plot(swiv,q5s,'k',[-pi,pi],[170*pi/180,170*pi/180],'r',[-pi,pi],[-170*pi/180,-170*pi/180],'r')
% subplot(8,1,6);
% plot(swiv,q6s,'k',[-pi,pi],[120*pi/180,120*pi/180],'r',[-pi,pi],[-120*pi/180,-120*pi/180],'r')
% subplot(8,1,7);
% plot(swiv,q7s,'k',[-pi,pi],[175*pi/180,175*pi/180],'r',[-pi,pi],[-175*pi/180,-175*pi/180],'r')
% 
% subplot(8,1,8);
% plot(swiv(b_qwithinLimits(1,:,1)),zeros(1,length(swiv(b_qwithinLimits(1,:,1)))),'xg',swivel,0,'o');
% axis([-4,4,-1,1])

% 1st config
% q1

% c2 = numOrDenShoulder(3,3,swivel);
% % boolVec = (c2~=1 & c2~=-1);
% % q1 = boolVec.*atan2(numOrDenShoulder(2,3,swivel),numOrDenShoulder(1,3,swivel)) + 0;
% % q3 = boolVec.*atan2(numOrDenShoulder(3,2,swivel),-numOrDenShoulder(3,1,swivel)) + (~boolVec).*atan2(numOrDenShoulder(2,1,swivel),numOrDenShoulder(2,2,swivel));
% % q2 = boolVec.*( (sin(q1)~=0).*atan2(numOrDenShoulder(2,3,swivel)./sin(q1),numOrDenShoulder(3,3,swivel)) + (sin(q1)==0).*atan2(numOrDenShoulder(1,3,swivel)./cos(q1),numOrDenShoulder(3,3,swivel))) + 0; % ATTENTION : not able to equal pi if not boolVec, but should
% 
% c6 = numOrDenWrist(2,3,swivel);
% % boolVec = (c6~=1 & c6~=-1);
% % q5 = boolVec.*atan2(-numOrDenWrist(3,3,swivel),numOrDenWrist(1,3,swivel)) + 0;
% % q7 = boolVec.*atan2(numOrDenWrist(2,2,swivel),-numOrDenWrist(2,1,swivel)) + (~boolVec).*atan2(numOrDenWrist(3,1,swivel),numOrDenWrist(3,2,swivel));
% % q6 = boolVec.*( (cos(q5)~=0).*atan2(numOrDenWrist(1,3,swivel)./cos(q5),numOrDenWrist(2,3,swivel)) + (cos(q5)==0).*atan2(-numOrDenWrist(3,3,swivel)./sin(q5),numOrDenWrist(2,3,swivel)) ) + 0;
% 
% if (c2~=1 && c2~=-1)
%     q1 = atan2(numOrDenShoulder(2,3,swivel),numOrDenShoulder(1,3,swivel));
%     q3 = atan2(numOrDenShoulder(3,2,swivel),-numOrDenShoulder(3,1,swivel));
%     if (sin(q1)~=0)
%         q2 = atan2(numOrDenShoulder(2,3,swivel)./sin(q1),numOrDenShoulder(3,3,swivel));
%     else
%         q2 = atan2(numOrDenShoulder(1,3,swivel)./cos(q1),numOrDenShoulder(3,3,swivel));
%     end
% else
%     q1 = 0;
%     q3 = atan2(numOrDenShoulder(2,1,swivel),numOrDenShoulder(2,2,swivel));
%     q2 = 0;
% end
% 
% if (c6~=1 && c6~=-1)
%     q5 = atan2(-numOrDenWrist(3,3,swivel),numOrDenWrist(1,3,swivel));
%     q7 = atan2(numOrDenWrist(2,2,swivel),-numOrDenWrist(2,1,swivel));
%     if (cos(q5)~=0)
%         q6 = atan2(numOrDenWrist(1,3,swivel)./cos(q5),numOrDenWrist(2,3,swivel));
%     else
%         q6 = atan2(-numOrDenWrist(3,3,swivel)./sin(q5),numOrDenWrist(2,3,swivel));
%     end
% else
%     q5 = 0;
%     q7 = atan2(numOrDenWrist(3,1,swivel),numOrDenWrist(3,2,swivel));
%     q6 = 0;
% end


Q = permute(qq(:,ind_0,:),[1 3 2])';
% D_Phi1 = 



end