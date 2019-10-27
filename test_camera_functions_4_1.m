% >>>>>>> Test Bundle Adjustment using LM Algorithm with 3 cameras <<<<<<<<
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
%Rt2 = [eul2dcm(deg2rad([0 0 0])) [1 0 0]'];
C{2} = cameraMoveRelative1(C{1},Rt2);

% ----------------------------------------------------------------
% (3) Get initial estimate of camera motions and structure using 2
%     consecutive frames.
% ----------------------------------------------------------------
X = [];         % initial scene points
id_X = [];      % initial scene point indecies in <W1> for checking after BA
M = [];         % Correspondence matrix for Bundle Adjustment (rows --> camera, colums --> points, value --> index in C.pixel)

col_all = 'rbygkcm';
marker_all = 'x+.*o^sd';
for i=2:length(C)
    % (a) Capture images
    if i==2
        C{1} = cameraCapture1(C{1},W1);     % First camera only
    end
    C{i} = cameraCapture1(C{i},W1);         % i-th camera
    % (b) Matching (index of correspondence)
    [id_global,id_C1,id_C2] = getPixelCorrespondence(C{i-1},C{i},0);
    
    
    
    
    
    
    % plot
    x1 = C{i-1}.pixel(:,id_C1);
    x2 = C{i}.pixel(:,id_C2);
    % choose the first point in both camera
    p1_a = x1(:,1);
    p2_a = x2(:,1);
    % compute the epipoles
    T = inv([C{i}.Rt;0 0 0 1]);
    ep1 = C{i}.K*T(1:3,:)*cart2homog(C{i-1}.Rt(:,4));
    T = inv([C{i-1}.Rt;0 0 0 1]);
    ep2 = C{i-1}.K*T(1:3,:)*cart2homog(C{i}.Rt(:,4));
    if isEqual(ep1(3),0)
        p1_b = p1_a + ep1(1:2);
    else
        p1_b = homog2cart(ep1);
    end
    if isEqual(ep2(3),0)
        p2_b = p2_a + ep2(1:2);
    else
        p2_b = homog2cart(ep2);
    end
    % move the end points outside the image limits
    Eps = 1e3;
    v1 = p1_b-p1_a;
    p1_b = p1_a+Eps*v1;
    p1_a = p1_a-Eps*v1;
    
    v2 = p2_b-p2_a;
    p2_b = p2_a+Eps*v2;
    p2_a = p2_a-Eps*v2;
    
    
    figure(10), clf
    set(gcf, 'color', [1 1 1])
    for j=1:size(x1,2)
        col = col_all(randomsample(length(col_all),1));
        marker = marker_all(randomsample(length(marker_all),1));
        subplot(1,2,1)
        plotPoints(x1(:,j),[col,marker]);
        if j==1
            hold on
        elseif j==size(x1,2)
            plotPoints([p1_a p1_b],'k-');
            hold off
            axis equal
            axis([0 C{i}.res(1) 0 C{i}.res(2)])
            title('camera1')
        end
        subplot(1,2,2)
        plotPoints(x2(:,j),[col,marker]);
        if j==1
            hold on
        elseif j==size(x1,2)
            plotPoints([p2_a p2_b],'k-');
            hold off
            axis equal
            axis([0 C{i}.res(1) 0 C{i}.res(2)])
            title('camera2')
        end
    end
    

    
    
    
    % (c) Essential Matrix
    E = getEssentialMatrix(C{i-1},C{i},id_C1,id_C2);
    % (d) Recover Motion (all cases)
    Rt_all = essentialMat2projectionMats(E);
    Rt_all = Rt2;
    % (e) Recover Structure (Triangulation)
    [Xt,C{i}] = triangulate3DPoint1(C{i-1},C{i},id_C1,id_C2,Rt_all);
    
    % (f) Update <X>, <X_ind>, and <M> using <id_global>
    % ...
%     if i==3
%         [ind_old,IA,IB] = intersect(id_X,id_global);
%         X(:,IA)
%         Xt(:,IB)
%     end
%     
    [id_new,ind_id_global] = setdiff(id_global,id_X);   % new points and their index in <id_global>
    [~,ind1,ind2] = intersect(id_global,id_X);          % for old points seen in C{i}
    X = [X, Xt(:,ind_id_global)];
    id_X = [id_X, id_new];
    % Update <M>:
    new_cols = [zeros(size(M,1)-1,length(ind_id_global));id_C1(ind_id_global)';id_C2(ind_id_global)'];
    new_rows = zeros(1,size(M,2));
    new_rows(ind2) = ind1;
    M = [[M;new_rows] new_cols];
end

% ---------------------
% (6) Bundle Adjustment
% ---------------------
[C,X] = bundleAdujustment1(C,X,M);

% ------------
% (5) Plotting
% ------------
% (a) True Motion and Structure
figure(fid), clf, fid = fid+1;
set(gcf, 'color', [1 1 1])
%     Plot cameras
cameraPlot1(C{1},'C_1')
hold on
for i=2:length(C)
    cameraPlot1(C{i},['C_' num2str(i)])
end
%     Plot Points
%plotPoints(W1.point,'b.',10);                       % All points in <W1>
for i=1:length(C)
    %plotPoints(W1.point(:,C{i}.pixel_id),'ro',10);  % points seen in C{i}
end

% From before...
% plotPoints(Xt,'kx',10);
% plotPoints(W1.point(:,id_global),'ro',10);

plotPoints(X,'kx',10);
plotPoints(W1.point(:,id_X),'ro',10);

hold off
axis equal
%axis(W.limits)
xlabel('x'), ylabel('y'), zlabel('z')
grid on
