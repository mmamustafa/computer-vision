function C = cameraGenerate1(K,Rt,res)
% -------------------------------------------------------------------------
% This function generates a pinhole camera.
%
% Inputs:
%   <K>         (3,3)   Camera caibration.
%   <Rt>        (3,4)   [R|t] of the camera center wrt {W}.
%   <res>       (1,2)   number of cols and rows, respectively
%
% Outputs:
%   <C>         (1,1)	Structure with input parameters
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, June 2017
% Modifications:    ...
% -------------------------------------------------------------------------

% Default values
% http://www.mrpt.org/tutorials/programming/miscellaneous/kinect-calibration/
if nargin<3
    res = [640 480];
    if nargin<2
        Rt = [eye(3) [0 0 0]'];
        if nargin<1
            K = [   520     0       320;
                    0       490     240;
                    0       0       1];
        end
    end
end

% create the data structure
C.K = K;
C.Rt = Rt;              % homogenous coordinate
C.res = res;            % camera resolution [cols, rows]
C.pixel = [];           % detected pixels (2,nPix)
C.pixel_id = [];        % ID of each detected pixel (1,nPix)
C.noise_std = 1;        % standard deviation of zero-mean noise
C.Rt_est = [eye(3) [0 0 0]'];   % estimated [R|t]
return