function W = world3DpointsGenerate1(np,limits)
% -------------------------------------------------------------------------
% This function generates a 3D world with random points wrt {W}.
%
% Inputs:
%   <np>        (1,1)   number of points to generate.
%   <limits>	(1,6)   limits in x,y,z.
%
% Outputs:
%   <W>         (1,1)	Structure with world parameters parameters
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, June 2017
% Modifications:    ...
% -------------------------------------------------------------------------

% Default values
if nargin<2
    limits = [-2 2 -2 2 -2 2];   % x, y, z
    if nargin<1
        np = 1e3;    % number of points to generate
    end
end

% create the data structure
W.limits = limits;
W.point = uniSampleND(limits,np);

return