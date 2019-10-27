function [id_global,id_C1,id_C2] = getPixelCorrespondence(C1,C2,mis_perc)
% -------------------------------------------------------------------------
% Find pixel matches between <C1> and <C2> in terms of indecies.
%
% Inputs:
%   <C1>        (1,1)   Camera 1.
%   <C2>        (1,1)   Camera 2.
%   <mis_perc>  (1,1)   Mismatch percentage [0:1]
%
% Outputs:
%   <id_global> (1,n)   index in the <W1.point> (used mainly for plotting)
%   <id_C1>     (1,n)   index in <C1>.
%   <id_C2>     (1,n)   index in <C2>.
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, July 2017
% Modifications:    20 July 2017:   Allow mismatch with percentage of <mis_perc>
% -------------------------------------------------------------------------

% Default values
if nargin<3
    mis_perc = 0;   % no mismatch
    if nargin<2
        error('Bad parameters!')
    end
end

% Get correspondence
[id_global,id_C1,id_C2] = intersect(C1.pixel_id,C2.pixel_id);

% Create mismatch according to <mis_perc>
ind0 = randomsample(length(id_C1),round(length(id_C1)*mis_perc));
if ~isempty(ind0)
    id_C1(ind0) = randomsample(id_C1(ind0),length(ind0));
    id_C2(ind0) = randomsample(id_C2(ind0),length(ind0));
end
% % for testing RANSAC only (delete later)
% setdiff(1:length(id_C1),ind0)
return