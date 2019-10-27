clc, clear all,

I = imread('flower.jpg');
I_template = imread('flower_crop.jpg');
p = zeros(6,1); p(5:6) = [120 200]';

[p,I_roi,T_error]=LucasKanadeAffine(I,p,I_template);
p
T_error

figure(1),clf
imshow(I)

figure(2), clf
subplot(1,2,1)
imshow(I_template)
title('Template')
subplot(1,2,2)
imshow(I_roi)
title('Warped Image from I')