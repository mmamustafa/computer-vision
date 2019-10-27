function [C,X] = bundleAdujustment1(C,X,M)
% -------------------------------------------------------------------------
% Bundle Adjustment.
%
% Inputs:
%   <C>         (1,nc)	Set of nc cameras (each is a structure).
%   <X>         (3,np)  Set of np 3D points.
%   <M>         (3,xx)  Correspondence matrix:
%                           rows    --> camera,
%                           colums  --> points,
%                           value   --> index in C.pixel
%
% Outputs:
%   <C>         (1,nc)	Set of nc cameras (each is a structure).
%   <X>         (3,np)  Set of np 3D points.
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, August 2017
%
% References:
%   - Multiple View Geometry (pages: 597-)
%   - https://cseweb.ucsd.edu/classes/fa04/cse252c/manmohan1.pdf
% -------------------------------------------------------------------------

if nargin<3
    error('Bad parameters!')
end

nc = length(C);     % number of cameras
np = size(X,2);     % number of 3D points

% (1) Construct the initial parameter vector <P> such that:
%     P = [R_1', t_1', ..., R_nc', t_nc', X_1', ..., X_np']'



return