function [cog_EE, m_EE, ret] = vrep_getLoadData(clientID,vrep)


% retrieve all dummies handles and names in vrep scene
[ret,dummiesHandles,int,floats,dummyNames]=vrep.simxGetObjectGroupData(clientID,vrep.sim_object_dummy_type,0,vrep.simx_opmode_blocking);

% find the name of the center of gravity dummy
cog_EE_name = dummyNames{not(cellfun('isempty', strfind(dummyNames(:,1), 'cog_EE')))};

% retrieve mass
m_EE = str2num(cog_EE_name(8:end))/1000; % mass in kg (the name contains the mass, in g


% retrieve dummy handle
cog_EE_h = dummiesHandles(not(cellfun('isempty', strfind(dummyNames(:,1), 'cog_EE'))));
% retrieve cog position in parent frame
[ret,cog_EE]=vrep.simxGetObjectPosition(clientID,cog_EE_h,vrep.sim_handle_parent,vrep.simx_opmode_blocking);
cpt=1;
while (ret > 0 || cpt > 10)
    [ret,cog_EE]=vrep.simxGetObjectPosition(clientID,cog_EE_h,vrep.sim_handle_parent,vrep.simx_opmode_blocking);
    cpt = cpt+1;
    pause(0.001);
end

end