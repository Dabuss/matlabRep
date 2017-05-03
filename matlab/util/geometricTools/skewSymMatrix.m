function [M] = skewSymMatrix( u )
% Span the skew-symmetric matrix of vector u.
% Attention : u has to be normalized


M=[0 -u(3) u(2) ; u(3) 0 -u(1) ; -u(2) u(1) 0 ];


end

