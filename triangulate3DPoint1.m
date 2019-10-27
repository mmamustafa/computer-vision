function [X,C2] = triangulate3DPoint1(C1,C2,id_C1,id_C2,Rt_all)
% -------------------------------------------------------------------------
% Triangulate the 3D point <X> using correspondence from <C1> and <C2> and
% the correct <Rt> in <Rt_all>.
%
% Camera 1 is assumed to be at the origin, and <Rt_all> is the relative 
% pose of camera 2.
%
% Pixel correspondence are found using <ind1> and <ind2>.
% 
% The "correct" <Rt> is found by making sure the constructed 3D point is in
% front of both cameras.
%
% Works with 2 cameras only.
%
% Iterative DLT method as shown in:
%   http://users.cecs.anu.edu.au/~hartley/Papers/triangulation/triangulation.pdf
%
% Inputs:
%   <C1>        (1,1)   Camera 1.
%   <C2>        (1,1)   Camera 2.
%   <id_C1>     (1,np)  index of matched pixel in <C1>.
%   <id_C2>     (1,np)  index of matched pixel in <C2>.
%   <Rt_all>    (1,4)   Cell array that contains all 4 possible <Rt> from
%                       essential matrix corresponding to Relative pose of <C2>.
%                       Or:
%               (3,4)   [R|t].
%
% Outputs:
%   <X>         (3,np)  Reconstructed 3D point/s
%   <C2>        (1,1)   Camera 2 with correct estimation of [R|t] in {W}.
%                       (C2.Rt_est).
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, July 2017
%
% References:
%   - http://www.robots.ox.ac.uk/~az/tutorials/cvpr03_part1.pdf
%   - Multiple View Geometry (pages: 310-313)
% -------------------------------------------------------------------------

if nargin<5
    error('Bad parameters!')
end

np = length(id_C2);     % number of points
X = zeros(3,np);        % initialize the 3D points

% Get correspondence
x1 = C1.pixel(:,id_C1);
x2 = C2.pixel(:,id_C2);

% Construct <P1>. Always at the origin
P(:,:,1) = C1.K*[eye(3) [0 0 0]'];      % <P1>

% Find the correct [R|t] for the second camera and <P2>
if ~iscell(Rt_all)
    % one one case to consider
    T_temp = inv([Rt_all;0 0 0 1]);     % check "cameraCapture2.m" to justify <inv>
    P(:,:,2) = C2.K*T_temp(1:3,:);      % <P2>
    Rt2 = Rt_all;
else
    % consider 4 cases.
    for j=1:length(Rt_all)
        T_temp = inv([Rt_all{j};0 0 0 1]);	% check "cameraCapture2.m" to justify <inv>
        P(:,:,2) = C2.K*T_temp(1:3,:);      % <P2>
        % Reconstruct only one 3D points
        Xt = iterativeLinearTriangulation(P,[x1(:,1) x2(:,1)]);
        % make sure the point is in front of both cameras
        temp1 = P(:,:,1)*cart2homog(Xt);	% project on <C1>
        temp2 = P(:,:,2)*cart2homog(Xt);	% project on <C2>
        if temp1(end)>0 && temp2(end)>0
            Rt2 = Rt_all{j};
            break
        end
    end
end

% Reconstruct all 3D points
for i=1:np
    X(:,i) = iterativeLinearTriangulation(P,[x1(:,i) x2(:,i)]);
end

% Transform <X> to {W} using <C1.Rt_est>.
% Note: This step may change the norm of translation from unit vector.
T1 = [C1.Rt;0 0 0 1];
T2 = T1*[Rt2;0 0 0 1];
X = homog2cart(T1*cart2homog(X));
% Update <C2.Rt_est>
C2.Rt_est = T2(1:3,:);
return