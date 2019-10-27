function [E,inliers] = getEssentialMatrix_ransac(C1,C2,id_C1,id_C2)
% -------------------------------------------------------------------------
% This function is similar to "getEssentialMatrix" but it assumes there is
% mismatch in correspondence. Therefore, RANSAC is applied.
%
% Inputs:
%   <C1>        (1,1)   Camera 1.
%   <C2>        (1,1)   Camera 2.
%   <id_C1>     (1,n)   index of matched pixel in <C1>
%   <id_C2>     (1,n)   index of matched pixel in <C2>
%   
%
% Outputs:
%   <E>         (3,3)   Essential Matrix (rank 2).
%   <inliers>   (1,m)   index of inliers in <id_C1> and <id_C2>.
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, July 2017
% Modifications:    ...
% -------------------------------------------------------------------------

% Default values
if nargin<4     % this is different than "getEssentialMatrix" because we assume we have the matching already
    error('Bad parameters!')
end

% Initial parameters
k = 0;              % counter
ni = 0;             % number of inliers
nx = length(id_C1); % Total number of indecies
nm = 8;             % minimum number of points.
p = 0.99;           % probability of choosing inliers set.
thr = 1e-2;         % Threshold
stop = 0;           % stopping condition
inliers_temp = zeros(1,nx);
if nx<nm
    error('RANSAC: minimum number of required points is available!')
end
while ~stop
    % (1) Incremet the counter
    k = k+1;
    % (2) Choose 8 (minimum) indecies randomly
    ind0 = randomsample(nx,nm);
    % (3) Fit the Essential matrix model
    [~,F] = getEssentialMatrix(C1,C2,id_C1(ind0),id_C2(ind0));
    % (4) Find the number of inliers and their indecies
    for i=1:nx
        %inliers_temp(i) = (abs(cart2homog(C1.pixel(:,id_C1(i)))'*F*cart2homog(C2.pixel(:,id_C2(i))))<thr);
        inliers_temp(i) = sum((cart2homog(C1.pixel(:,id_C1(i))) - F*cart2homog(C2.pixel(:,id_C2(i)))).^2);
    end
    inliers_temp
    rrrr
    ni_temp = sum(inliers_temp);
    % (5) Consider a better model
    if ni_temp>ni
        inliers = inliers_temp;
        ni = ni_temp;
        nk = log(1-p)/log(1-(ni/nx)^nm);
    end
    % (6) Stopping condition
    if k>nk
        stop = 1;
    end
end
% (7) Compute the best model with ALL inliers
E = getEssentialMatrix(C1,C2,id_C1(inliers),id_C2(inliers));

return