function w = rotationVectorNormalized(w)
% -------------------------------------------------------------------------
% This function normalizes the rotation vector <w> in so(3) such that:
%       ||w||<pi.
% 
% Input:
%   <w>     (3,1)   Rotation vector in so(3). Angle-axis form.
%
% Output:
%   <w>     (3,1)   Normalized rotation vector in so(3). Angle-axis form.
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, July 2017
% Modifications:    ...
%
% References:
%   - Multiple View Geometry (pages: 624).
% -------------------------------------------------------------------------

theta = sqrt(sum(w.^2));
w = reduce_angle_fullCircle(theta)*w/theta;
return