function J_0 = getGeometricJacobianIiwa(config, tcp)
% gemotric jacobian in base 0
% angles are given in

    Hs = getFkIiwa(config, tcp);

    J_0 = zeros(6,7);
    for i = 1 : 7
        H_0_i = Hs{i};
        
        zi_0 = H_0_i(1:3,3); %vector zi seen in R0
        EOi_0 = (H_0_i(1:3,4) - Hs{8}(1:3,4))*0.001; %vector EOi seen in R0 (in m)
        
        % ith column
        J_0(:,i) = [cross(EOi_0,zi_0); zi_0];

    end



end