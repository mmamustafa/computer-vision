function [R,Q] = rq(A)
% -------------------------------------------------------------------------
% RQ decomposition, such that R*Q = A
%
% Inputs:
%   <A>         (m,n)   Input matrix.
%
% Outputs:
%   <R>         (m,n)   Upper triangular matrix.
%   <Q>         (n,n)   Unitary matrix.
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, April 2020
% Modifications:    ...
%
% References:
%   - https://leohart.wordpress.com/2010/07/23/rq-decomposition-from-qr-decomposition/
% -------------------------------------------------------------------------

[Q,R] = qr(rot90(A,3));
R = rot90(R,2)';
Q = rot90(Q);
return