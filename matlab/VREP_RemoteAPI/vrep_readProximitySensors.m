function [ detectionStates, detectedPoints, detectedObjectHandles, detectedSurfacesNormalVector ] = vrep_readProximitySensors(clientID,vrep, proximitySensors_handles)
% retrieves readings from sensors object handles. distance unit is meter

nprox = length(proximitySensors_handles);

detectionStates = zeros(nprox,1);
detectedPoints = zeros(nprox,3);
detectedObjectHandles = zeros(nprox,1);
detectedSurfacesNormalVector = zeros(nprox,3);


for i=1:length(proximitySensors_handles)
    [ret,detectionStates(i),detectedPoints(i,:),detectedObjectHandles(i),detectedSurfacesNormalVector(i,:)] = vrep.simxReadProximitySensor(clientID,proximitySensors_handles(i),vrep.simx_opmode_streaming);
%     pause(0.1);
end



end

