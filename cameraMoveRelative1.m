function C = cameraMoveRelative1(C,Rt)
% -------------------------------------------------------------------------
% This function moves the camera to new position according to <Rt> wrt
% previous camera position.
%
% Inputs:
%   <C>         (1,1)   Camera structure.
%   <Rt>        (3,4)   [R|t] of the camera center wrt currect camera pose.
%
% Outputs:
%   <C>         (1,1)	Camera structure.
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, June 2017
% Modifications:    ...
% -------------------------------------------------------------------------

% Default values
if nargin<2
    Rt = [eye(3) [0 0 0]'];
    if nargin<1
        C = cameraGenerate1();      % generate simple camera
    end
end

% update camera position
T = [C.Rt;0 0 0 1]*[Rt;0 0 0 1];    % Transformation in homogenous coordinate
C.Rt = T(1:3,:);
return