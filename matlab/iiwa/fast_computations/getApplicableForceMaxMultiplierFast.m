function lambda = getApplicableForceMaxMultiplierFast(config, tcpEulerZYX, m_EE, transG_EE, f)
% returns the maximum magnitude of f that is applicable for the tcp
% situated at tcpEulerZYX. spatial force units are [N;N;N;Nm;Nm;Nm]

Tmax = [320;320;176;176;110;40;40];
Tmin = -Tmax;

Tg = getGravityTorquesIiwaFast(config, m_EE, transG_EE);
Tf = getTorquesFromSpatialForceIiwaFast(config, tcpEulerZYX, f);

lambda = min(abs((Tf>0).*(Tmax-Tg)./Tf + (Tf<0).*(Tmin-Tg)./Tf),999999);


end


