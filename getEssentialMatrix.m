function [E,F] = getEssentialMatrix(C1,C2,id_C1,id_C2)
% -------------------------------------------------------------------------
% This function returns the essential matrix constraining two cameras using
% Direct Linear Transform (DLT).
%
% 8-point algorithm.
%
% Only uses the following from each camera structure:
%   <K>         The camera calibration
%   <pixel>     The pixels detected
%   <pixel_id>  The ID's of each detected pixel for data association.
%
% Inputs:
%   <C1>        (1,1)   Camera 1.
%   <C2>        (1,1)   Camera 2.
%   <id_C1>     (1,n)   index of matched pixel in <C1>
%   <id_C2>     (1,n)   index of matched pixel in <C2>
%
% Outputs:
%   <E>         (3,3)   Essential Matrix (rank 2).
%   <F>         (3,3)   Fundametal Matrix
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, June 2017
% Modifications:    ...
%
% References:
%   - http://mi.eng.cam.ac.uk/~cipolla/publications/contributionToEditedBook/2008-SFM-chapters.pdf
%   - https://en.wikipedia.org/wiki/Essential_matrix
%   - Multiple View Geometry
% -------------------------------------------------------------------------

% Default values
if nargin<3
    [id_C1,id_C2] = getPixelCorrespondence(C1,C2);
    if nargin<2
        error('Bad parameters!')
    end
end

% Get correspondence
x1 = C1.pixel(:,id_C1);
x2 = C2.pixel(:,id_C2);

% Normalize the pixels
[N1,x1_hat] = getNormalizationMatrix(x1);
[N2,x2_hat] = getNormalizationMatrix(x2);

% Constrcut A matrix such that A*f = 0.
np = length(x1_hat);    % number of matched points
if np<8
    error('8-point Algorithm requires at least 8 matched points!')
end

A = zeros(np,9);
for i=1:np
    A(i,:) = reshape(x2_hat(:,i)*x1_hat(:,i)',1,[]);
end

% Compute the null space of A usin SVD
[~,~,V] = svd(A);       % do not use 'econ'
f = V(:,end);           % null space of A

% Construct the normalized fundamental matrix <F_hat>
F_hat = reshape(f,3,[])';

% Make sure <F_hat> has rank 2
[U,S,V] = svd(F_hat);
F_hat = U*diag([S(1,1) S(2,2) 0])*V';

% Un-normalize <F_hat>
% It should satisfy Epipolar constraint:   cart2homog(x1(:,i))'*F*cart2homog(x2(:,i)) ~ 0.
F = N1'*F_hat*N2;

% Compute the Essential matrix using camera calibration
E = C1.K'*F*C2.K;

return