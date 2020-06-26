% >>>>>> Test Computing camera parameters from 2D-3D correspondence <<<<<<<
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

clc, clear all, fid = 1;

% -----------------------------
% (1) Generate the world Points
% -----------------------------
W1 = world3DpointsGenerate1(1e2);	% create the world

% --------------------
% (2) Generate cameras
% --------------------
% (a) Camera 1
C{1} = cameraGenerate1();             % create the camera
Rt1 = [eul2dcm(deg2rad([90 -90 0])) [-2 0 0]'];
C{1} = cameraMoveAbsolute1(C{1},Rt1);
C{1}.Rt_est = C{1}.Rt;                  % only for the initial frame
% (b) Camera 2
Rt2 = [eul2dcm(deg2rad([0 45 20])) [1 0 1]'];
C{2} = cameraMoveRelative1(C{1},Rt2);
% (c) Camera 3
Rt3 = [eul2dcm(deg2rad([-10 40 10])) [1.5 0 0.5]'];
C{3} = cameraMoveRelative1(C{2},Rt3);

% ----------------------------------------------------------------
% (3) Capture images and estimate camera parameters for each scene.
% ----------------------------------------------------------------
for i=1:length(C)
    % (a) Capture images
    C{i} = cameraCapture1(C{i},W1);
    % (b) Compute camera parameters assuming the Map is known
    [K,Rt] = getCameraParamFrom2d3d(C{i},W1);
    disp('Two matrices should be close to each other:')
    C{i}.Rt
    Rt
    disp('---------')
    
end
