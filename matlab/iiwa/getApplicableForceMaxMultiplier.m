function lambda = getApplicableForceMaxMultiplier(config, tcpEulerZYX, m_EE, transG_EE, f)
% returns the maximum magnitude of f that is applicable for the tcp
% situated at tcpEulerZYX. spatial force units are [N;N;N;Nm;Nm;Nm]

Tmax = [320;320;176;176;110;40;40];
Tmin = -Tmax;

Tg = getGravityTorquesIiwa(config, m_EE, transG_EE);
Tf = getTorquesFromSpatialForceIiwa(config, tcpEulerZYX, f);

lambdas = (abs(Tg)<Tmax).* (abs((Tf>0).*(Tmax-Tg)./Tf + (Tf<0).*(Tmin-Tg)./Tf));
lambda = min(lambdas,999999);

end

