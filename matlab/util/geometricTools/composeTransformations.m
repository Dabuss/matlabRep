function [ Hout ] = composeTransformations(H1,H2)
%obsolete (less quick than normal matrix multiplication)

% R1 = H1(1:3,1:3);
% R2 = H2(1:3,1:3);
% p2 = H2(1:3,4);

Hout = [H1(1:3,1:3)*H2(1:3,1:3), H1(1:3,1:3)*H2(1:3,4);0,0,0,1];

end

