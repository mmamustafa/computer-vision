function [N,pts_hat] = getNormalizationMatrix(pts)
% -------------------------------------------------------------------------
% Normalize points (pixels) to have mean of 1 and standard deviation of 1
% in each dimension.
%
% Each dimesnion is assumed independent.
%
% Works in 2D only (for now)
% 
% Input:
%   <pts>       (2,np)	points in 2D
%
% Output:
%   <N>         (3,3)   Transformation matrix in homogoenous coordinate
%   <pts_hat>   (3,np)  Homogenuous Normalized points (N*homog(pts))
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, June 2017
% Modifications:    ...
%
% References:
%   - http://www.maths.lth.se/matematiklth/personal/calle/datorseende13/notes/forelas3.pdf
%
% -------------------------------------------------------------------------

[nd,~] = size(pts);     % number of dimensions
mu = mean(pts,2);       % mean
s = zeros(1,nd);        % initialization
for i=1:nd
    s(i) = std(pts(i,:));   % standard deviation
end
N = [1/s(1)     0       -mu(1)/s(1);
    0           1/s(2)  -mu(2)/s(2);
    0           0       1];
if nargout>1
    pts_hat = N*cart2homog(pts);
end
return