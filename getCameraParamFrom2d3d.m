function [K,Rt] = getCameraParamFrom2d3d(C,W)
% -------------------------------------------------------------------------
% This function returns the camera parameters (intrinsic and extrinsic)
% from a set of 2D-3D correspondence using Direct Linear Transform (DLT).
%
% Only uses the following from each camera structure:
%   <K>         The camera calibration (if exists)
%   <pixel>     The pixels detected
%   <pixel_id>  The ID's of each detected pixel for data association.
%
% Inputs:
%   <C>         (1,1)   Camera.
%   <W>         (1,1)   World structure (map)
%
% Outputs:
%   <K>         (3,3)   Camera caibration.
%   <Rt>        (3,4)   [R|t] of the camera center wrt {W}.
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, April 2020
% Modifications:    ...
%
% References:
%   - Multiple View Geometry (pp.153-158, 178-180)
%   - https://www.uio.no/studier/emner/matnat/its/nedlagte-emner/UNIK4690/v16/forelesninger/lecture_5_2_pose_from_known_3d_points.pdf
% -------------------------------------------------------------------------

% Default values
if nargin<2
    error('Bad parameters!')
end

np = length(C.pixel_id);	% number of matched points
if np<6
    error('At least 6 2D-3D correspondences are required!')
end

% Extract 2D-3D correspondence
x = C.pixel;
X = W.point(:, C.pixel_id);

% Normalize the 2D and 3D points
[Nx,x_hat] = getNormalizationMatrix(x);
[NX,X_hat] = getNormalizationMatrix(X);

% Constrcut A matrix such that A*p = 0.
A = zeros(2*np,12);
for i=1:np
    ind_A = 2*(i-1) + 1;    % first equation index (row) in A
    % equation 1
    A(ind_A,1:4) = X_hat(:,i)';
    A(ind_A,9:end) = -x_hat(1,i)*X_hat(:,i)';
    % equation 2
    A(ind_A+1,5:8) =  X_hat(:,i)';
    A(ind_A+1,9:end) = -x_hat(2,i)*X_hat(:,i)';
end

% Compute the null space of A usin SVD
[~,~,V] = svd(A);       % do not use 'econ'
p = V(:,end);           % null space of A

% Construct the normalized camera matrix <P_hat>
P_hat = reshape(p,4,[])';

% Denormalize <P_hat> to get <P>
P = Nx\(P_hat*NX);

% Ensure input to RQ decompostion has +ve determinant
P = sign(det(P(:,1:3))) * P;    % Check sign(M)

% Compute camera center <c> in real world
[~,~,V] = svd(P);           % do not use 'econ'
c = homog2cart(V(:,end));	% null space of P since P*c = 0

% Extract <M> from <P> and apply RQ decomposition
M = P(:,1:3);               % M = K*R,   det(M) > 0
[K_hat,R_t] = rq(M);

% Make sure <K_hat> has positive diagonal elements
D = diag(sign(diag(K_hat)));    % D*D = eye(3) --> K_hat*R_t = K_hat*D*D*R_t
R_t = D * R_t;
K_hat = K_hat * D;
K = K_hat./K_hat(end);

% Return the camera pose in the world
% (this is because my convention to match C.Rt)
% No need to compute translation from world to camera `t = -R*c`
Rt = [R_t' c];
return