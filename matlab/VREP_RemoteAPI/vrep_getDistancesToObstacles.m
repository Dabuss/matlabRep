function distances_values = vrep_getDistancesToObstacles(clientID, vrep, distances_handles)
% retreive distances values from distance object handles distances are
% given in meters

ndistances = length(distances_handles);
distances_values = zeros(ndistances,1);

 for i=1:ndistances
%      [ret, distances_values(i)] = vrep.simxReadDistance(clientID, distances_handles(i), vrep.simx_opmode_streaming);
     [ret, d] = vrep.simxReadDistance(clientID, distances_handles(i), vrep.simx_opmode_streaming);
     distances_values(i) = d;
%      pause(0.1)
 end

end

