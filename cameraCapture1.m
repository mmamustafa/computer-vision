function C = cameraCapture1(C,W)
% -------------------------------------------------------------------------
% This function captures the scene in front of the camera.
%
% Inputs:
%   <C>         (1,1)   Camera structure.
%   <W>         (1,1)   World structure.
%
% Outputs:
%   <C>         (1,1)   Camera structure.
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, June 2017
% Modifications:    ...
% -------------------------------------------------------------------------

% Default values
if nargin<2
    error('Bad parameters!')
end

% simple linear projection
T = inv([C.Rt;0 0 0 1]);
t0 = C.K*T(1:3,:)*cart2homog(W.point);

% keep all point when <w> is greater than 0
% (point is in front of the camera).
ind0 = t0(3,:)>=0;

% convert to cartesian
t0 = homog2cart(t0);

% add gaussian noise
t1 = t0 + C.noise_std*randn(size(t0));

% form pixels
t1 = round(t1);

% keep points inside the camera limits
ind1 = t1(1,:)>0 & t1(1,:)<=C.res(1) & t1(2,:)>0 & t1(2,:)<=C.res(2);
ind = ind0 & ind1;
C.pixel = t1(:,ind);
C.pixel_id = find(ind);

return