function [Tg] = getGravityTorquesIiwa(config, m_EE, transG_EE)
% Computes the gravity torques of the robot, when its base is set on the
% floor based on an estimation of the masses and centers of gravity data

% from calibration :
% V_0 = [4.0000    4.4899    4.5166    3.7256    1.8208    3.3363    0.5739       0    2.9292    2.9767    1.2384    0.5334   -1.7781+20   -0.8745    -50.0000 -119.4238      -6.5391   59.0727    2.5454  -39.2055    0.1309  -96.7197  -0.0005  -134.1010   -1.5900  -74.1855  -26.2734   -3.2557]';
% % From article :
% V_0 = [[3.94781 4.50275 2.45520 2.61155 3.41000 3.38795 0.35432], -[-0.00351 -0.00767 -0.00225 0.00020 0.00005 0.00049 -0.03466 0.00160 0.16669 -0.03492 -0.05268 -0.00237 0.02019 -0.02324]*1000,[ -0.03139 -0.00355 -0.02652 0.03818 -0.21134 -0.02750 0.07138-0.126]*1000 ]';

% Tg = zeros(7,1);

g_0 = [0,0,-9.81]'; % vector of gravity in 0

m = [4.0000;    4.4899;    4.5166;    3.7256;    1.8208;    3.3363;    0.5739+m_EE];
mT = flipud(cumsum(flipud(m)));

xG = [0;    2.9292;    2.9767;    1.2384;    0.5334;   -1.7781+20;   (m(7)*(-0.8745)+m_EE*transG_EE(1))/(m_EE+m(7))];
yG = [-50.0000; -119.4238;  -6.5391;   59.0727;    2.5454;  -39.2055;  (m(7)*(0.1309)+m_EE*transG_EE(2))/(m_EE+m(7))];
zG = [-96.7197;  -0.0005;-134.1010;   -1.5900;  -74.1855;  -26.2734;   (m(7)*(-3.2557)+m_EE*transG_EE(3))/(m_EE+m(7))];

Gi_i = [xG,yG,zG]; % centers of links i seen from local reference frames i



Hs = getFkIiwa(config,zeros(1,6));


for i=7:-1:1
    Gi_0h = Hs{i}*[Gi_i(i,:),1]'; % homogeneous position of center of gravity of link i seen in 0
    Gi_x_mi(i,:) = m(i) * Gi_0h(1:3); % mass-i-ponderated gravity center position of link i (in 0)
    
    GTs(i,:) = sum(Gi_x_mi(i:7,:),1)/mT(i); % axis i carries links i, i+1, ... 7
    
    Oi = Hs{i}(1:3,4); % position of Oi (in (0))
    zi = Hs{i}(1:3,3); % vector zi (in (0))
    
    OiGti = (GTs(i,:)' - Oi)*0.001; % vector OiGti (in (0)), in meters
    
    Pti = mT(i)*g_0; % weight (in N)
    % computation of the moment around zi
    Tg(i) = sum(cross(OiGti,Pti).*zi); %(in Nm)
    
    
end

end

