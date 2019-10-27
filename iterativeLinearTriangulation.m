function X = iterativeLinearTriangulation(P,x)
% -------------------------------------------------------------------------
% Linear Triangulation method using Direct Linear Transform (DLT).
%
% Work with multiple cameras and a single point.
%
% Inputs:
%   <P>         (3,4,nf)    Projection matrices of <n> frames.
%   <x>         (2,nf)      pixel in each frame.
%
% Output:
%   <X>         (3,1)       Reconstructed 3D point
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, July 2017
% Modifications:    ...
%
% References:
%   - http://users.cecs.anu.edu.au/~hartley/Papers/triangulation/triangulation.pdf
% -------------------------------------------------------------------------

if nargin<2
    error('Bad parameters!')
end

nf = size(x,2);         % number of frames

% initialization
A = zeros(2*nf,4);
w = ones(1,nf);         % weight for each pixel

for j=1:10
    % construct A
    for i=1:nf
        A(2*i-1,:) = (x(1,i)*P(3,:,i) - P(1,:,i))/w(i);
        A(2*i,:) = (x(2,i)*P(3,:,i) - P(2,:,i))/w(i);
    end
    % Compute the null space of A using SVD
    [~,~,V] = svd(A);           % do not use 'econ'
    X = V(:,end);
    X = X/X(end);               % this step is important for stopping condition in case X(end) is negative
    % Recompute the weight
    for i=1:nf
        w(i) = P(3,:,i)*X;
    end
    % stopping condition
    if j>1
        if sum((X-X_old).^2)<1e-9
            break
        end
    end
    X_old = X;
end
% put <X> in Cartesian coordinate
X = X(1:3);
return
