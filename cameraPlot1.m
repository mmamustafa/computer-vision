function cameraPlot1(C,cName)
% -------------------------------------------------------------------------
% This function plots the camera wrt {W}
%
% Inputs:
%   <C>         (1,1)   Camera structure.
%   <cName>     (1,1)   camera name.
%
% Outputs:
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, June 2017
% Modifications:    ...
% -------------------------------------------------------------------------

% Default values
if nargin<2
    cName = [];
    if nargin<1
        error('Bad parameters!')
    end
end

% Define the robot scale
S = 0.1;

% Define the four vertices around the origin
X1 = S*[1 1 2;-1 1 2;-1 -1 2;1 -1 2;0 0 0]';

% Transform using T
X1 = homog2cart([C.Rt;0 0 0 1]*cart2homog(X1));

% plot the lines (almost all combinations)
ind1 = [1 2 3 4 1 2 3 4];
ind2 = [5 5 5 5 2 3 4 1];
x1 = X1(1,ind1);
x2 = X1(1,ind2);
y1 = X1(2,ind1);
y2 = X1(2,ind2);
z1 = X1(3,ind1);
z2 = X1(3,ind2);

line([x1;x2],[y1;y2],[z1;z2],'color','k','LineWidth',1)

% plot top triangle in different color (camera bottom)
patch(X1(1,[1 2 5]'),X1(2,[1 2 5]'),X1(3,[1 2 5]'),'g','EdgeColor','k','FaceAlpha',0.3)

% Put the robot name (always on the back of the robot)
if ~isempty(cName)
    pos = S*2;
    text_pos = homog2cart([C.Rt;0 0 0 1]*cart2homog([0;0;-(pos+S/2)]));
    text(text_pos(1),text_pos(2),text_pos(3),cName,'HorizontalAlignment','center');
end
return