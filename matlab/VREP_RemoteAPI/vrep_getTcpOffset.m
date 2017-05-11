function [tcp_offset, ret] = vrep_getTcpOffset(clientID,vrep,tcp_handle)
% retrieves the offset eulerZYX (mm,rad) corresponding to the tcp
tcp_offset = zeros(1,6);

[ret1,pos]=vrep.simxGetObjectPosition(clientID,tcp_handle,vrep.sim_handle_parent,vrep.simx_opmode_blocking);
tcp_offset(1:3) = pos*1000;

[ret2,eaXYZ]=vrep.simxGetObjectOrientation(clientID,tcp_handle,vrep.sim_handle_parent,vrep.simx_opmode_blocking);
tcp_offset(4:6) = Rmat_to_eaZYX(eaXYZ_to_Rmat(eaXYZ));

if (ret1==0) && (ret2==0)
    ret = 0;
else
    ret = 1;
end
end

