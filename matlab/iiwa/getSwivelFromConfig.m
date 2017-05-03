function [swivel,Ts] = getSwivelFromConfig(q)
%GETSWIVELFROMCONFIG Summary of this function goes here
%   Detailed explanation goes here
Ts = getFkIiwa(q,[0,0,0,0,0,0]);

S = Ts{1}(1:3,4);
E = Ts{4}(1:3,4);
W = Ts{6}(1:3,4);

sw = normalizeVector(W-S);
se = normalizeVector(E-S);

nSEW = cross(sw,se); % normalized normal vector of the SEW plane
ce = cross(nSEW,sw); % normalized vector along CE

u = vectorProjectedOnPlane([0,0,1]', sw); % projection of uz_world on the plane whose normal vector is sw. Beware, if sw is aligned with uz_world, uy_world will be taken instead

swivel = sign(dot(sw,cross(u,ce)))*angleBetweenUV(ce,u);


end

