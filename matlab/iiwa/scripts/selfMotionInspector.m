% SelfMotionInspector
% needs a vrep_store = vrepStore_miiwa(); beforehand




qinit = (rand(1,7)-1/2).*[170,120,170,120,170,120,175]*2*pi/180;
[swivel,FK] = getSwivelFromConfig(qinit);
pose = transformation_to_peaZYX(FK{8});

f = [0;0;-350;0;0;0];
def_ztcp_angle = 10*pi/180; % rad
def_xytcp_trans = 0.005; % m
maxDeflections = {def_ztcp_angle, def_xytcp_trans};

[admSwiv,qq] =computeIiwaSelfMotionAdmissibleRanges(...
    vrep_store.tcp_offset,...
    pose,...
    vrep_store.m_EE, ...
    vrep_store.cog_EE,...
    true,...
    f,...
    maxDeflections);

swiv = linspace(-pi,pi,size(qq,3));

while true
    figure(1);
    [inspectedSwivel,y] = ginput(1);
    
    if inspectedSwivel > pi
        break
    elseif inspectedSwivel < -pi
        qinit = (rand(1,7)-1/2).*[170,120,170,120,170,120,175]*2*pi/180;
        [swivel,FK] = getSwivelFromConfig(qinit);
        pose = transformation_to_peaZYX(FK{8});
        
        f = [0;0;-200;0;0;0];
        def_ztcp_angle = 10*pi/180; % rad
        def_xytcp_trans = 0.005; % m
        maxDeflections = {def_ztcp_angle, def_xytcp_trans};
        
        [admSwiv,qq] =computeIiwaSelfMotionAdmissibleRanges(...
            vrep_store.tcp_offset,...
            pose,...
            vrep_store.m_EE, ...
            vrep_store.cog_EE,...
            true,...
            f,...
            maxDeflections);
    end
    [m, ind] = min(abs(swiv-inspectedSwivel));
    
    vrep_store.setIiwaConfiguration(qq(:,1,ind));
end
