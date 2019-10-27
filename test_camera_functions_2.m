% Test RANSAC
% >>>>> NOT complete <<<<<

clc, clear all

% (0-a)	Simulate the World
W1 = world3DpointsGenerate1(1e2);	% create the world
% (0-b)  Simulate the first camera
C1 = cameraGenerate1();             % create the camera
Rt1 = [eul2dcm(deg2rad([90 -90 0])) [-2 0 0]'];
C1 = cameraMoveAbsolute1(C1,Rt1);
C1.Rt_est = C1.Rt;                  % only for the initial frame
% (0-c)  Simulate the first camera
Rt2 = [eul2dcm(deg2rad([0 45 20])) [1 0 1]'];
C2 = cameraMoveRelative1(C1,Rt2);

% (1) Capture images
C1 = cameraCapture1(C1,W1);
C2 = cameraCapture1(C2,W1);

% (2) Get index of correspondence (matching)
[id_global,id_C1,id_C2] = getPixelCorrespondence(C1,C2,0);

% (3) Get essential matrix
[E,F] = getEssentialMatrix(C1,C2,id_C1,id_C2);

% % For testing RANSAC only (delete later)
% for i=1:length(id_C1)
%     temp1 = distance_point_point(C1.pixel(:,id_C1(i)), homog2cart(F*cart2homog(C2.pixel(:,id_C2(i)))));
%     temp2 = cart2homog(C1.pixel(:,id_C1(i)))'*F*cart2homog(C2.pixel(:,id_C2(i)));
%     [temp1 temp2]
% end


% % for testing RANSAC only (Delete later)
% E1 = [0.6048    2.3386    0.6020
%    -0.0976   -0.8558    3.4166
%    -0.6047   -2.3385   -0.6015];
% F1 = [  0.000002236512957   0.000009178374257  -0.001760885052838
%   -0.000000383085508  -0.000003564267188   0.007950761071459
%   -0.001786610013959  -0.006854036981812  -0.428670804890433];
% return

%[E,inliers] = getEssentialMatrix_ransac(C1,C2,id_C1,id_C2);
%find(inliers)




% (4) Get all possible [R|t] from essential matrix
Rt_all = essentialMat2projectionMats(E);

% (5) Triangulation and Scene reconstruction
[X,C2] = triangulate3DPoint1(C1,C2,id_C1,id_C2,Rt_all);

C2.Rt
C2.Rt_est


% (6) Bundle Adjustment...



% (7) Plotting

% plot the world and the camera
figure(1), clf
cameraPlot1(C1,'C_1')
hold on
cameraPlot1(C2,'C_2')
plotPoints(W1.point(:,C1.pixel_id),'b.',10);
%plotPoints(W.point(:,C2.pixel_id),'rx',10);
plotPoints(X,'kx',10);

% common points in C1 and C2
plotPoints(W1.point(:,id_global),'ro',10);
hold off
axis equal
%axis(W.limits)
xlabel('x'), ylabel('y'), zlabel('z')
grid on

