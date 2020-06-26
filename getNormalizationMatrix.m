function [N,pts_hat] = getNormalizationMatrix(pts)
% -------------------------------------------------------------------------
% Normalize points (pixels) to have mean of 0 and standard deviation of 1
% in each dimension.
%
% Each dimesnion is assumed independent.
% 
% Input:
%   <pts>       (nd,np)	points in n-dimensions
%
% Output:
%   <N>         (nd+1,nd+1) Transformation matrix in homogoenous coordinate
%   <pts_hat>   (nd+1,np)  Homogenuous Normalized points (N*homog(pts))
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, June 2017
% Modifications:    April 2020: work for n-dimensions
%
% References:
%   - http://www.maths.lth.se/matematiklth/personal/calle/datorseende13/notes/forelas3.pdf
%
% -------------------------------------------------------------------------

% % Old approach only with 2D
% [nd,~] = size(pts);     % number of dimensions
% mu = mean(pts,2);       % mean
% s = zeros(1,nd);        % initialization
% for i=1:nd
%     s(i) = std(pts(i,:));   % standard deviation
% end
% N = [1/s(1)     0       -mu(1)/s(1);
%     0           1/s(2)  -mu(2)/s(2);
%     0           0       1];
% if nargout>1
%     pts_hat = N*cart2homog(pts);
% end


[nd,~] = size(pts);     % number of dimensions
mu = mean(pts,2);       % mean
s = std(pts, 0, 2);     % standard deviation
N = [diag(1./s), -mu./s;zeros(1, nd), 1];
if nargout>1
    pts_hat = N*cart2homog(pts);
end
return



