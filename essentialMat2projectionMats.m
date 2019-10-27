function Rt_all = essentialMat2projectionMats(E)
% -------------------------------------------------------------------------
% Get 4 possible projection matrices [R|t] given essetinal Matrix E.
% 5 degrees of ambiguity: 4 cases for rotation and translation, and one for
% scale.
%
% Only one of the four cases is correct and can be found by making sure the
% reconstructed 3D point is in front of both cameras.
%
% Inputs:
%   <E>         (3,3)   Essential matrix (rank 2).
%
% Outputs:
%   <Rt_all>    4 x (3,4)	cell array with 4 possible [R|t] matrices
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, June 2017
% Modifications:    ...
%
% References:
%   - http://mi.eng.cam.ac.uk/~cipolla/publications/contributionToEditedBook/2008-SFM-chapters.pdf
%   - https://en.wikipedia.org/wiki/Essential_matrix
%   - Multiple View Geometry (pages: 257-260)
% -------------------------------------------------------------------------

W = [0 -1 0;1 0 0;0 0 1];
[U,~,V] = svd(E);       % Generally, essential matrix should have rank of 2, 2 nonzero singular values are equal, and the last singular value should be zero 

u3 = U(:,3);
R1 = U*W*V';
R2 = U*W'*V';
% Make sure <R1> and <R2> are in SO(3), e.g. |R1| = |R2| = 1;
R1 = R1/det(R1);
R2 = R2/det(R2);

Rt1 = [R1 u3];
Rt2 = [R1 -u3];
Rt3 = [R2 u3];
Rt4 = [R2 -u3];

Rt_all = {Rt1,Rt2,Rt3,Rt4};
return