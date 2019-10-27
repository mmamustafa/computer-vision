clc, clear all

% number of points
np = 1e4;

% generate 2D points (pixel-like)
%x = uniSampleND([5 10 20 30],np);   % uniform
x = mvnrnd([4 10],[2 2;2 8],np)';   % gaussian

% Normalization matrix
N = getNormalizationMatrix(x);

% points after normalization
x_hat = homog2cart(N*cart2homog(x));

% test the mean and average distance (rmse) of x_hat
disp('Mean:')
mean(x_hat,2)
disp('Standard Deviation:')
sqrt(mean(sum(x_hat.^2)))


figure(1),clf
subplot(1,2,1)
plotPoints(x,'r.',10); axis equal
title('Before normalization')
subplot(1,2,2)
plotPoints(x_hat,'r.',10); axis equal
title('After normalization')
