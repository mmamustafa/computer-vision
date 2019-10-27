function C = cameraMoveAbsolute1(C,Rt)
% -------------------------------------------------------------------------
% This function moves the camera to new position according to <Rt> wrt {W}.
%
% Inputs:
%   <C>         (1,1)   Camera structure.
%   <Rt>        (3,4)   [R|t] of the camera center wrt {W}.
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
C.Rt = Rt;
return