% >>>>>>> Test Bundle Adjustment using LM Algorithm with 3 cameras <<<<<<<<
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

clc, clear all, fid = 1;
rand('seed', 0)
randn('seed', 0)

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
% (3) Get initial estimate of camera motions and structure using 2
%     consecutive frames.
% ----------------------------------------------------------------
X = [];         % initial scene points
id_X = [];      % initial scene point indecies in <W1> for checking after BA
M = [];         % Correspondence matrix for Bundle Adjustment (rows --> camera, colums --> points, value --> index in C.pixel)
for i=2:length(C)
    % (a) Capture images
    if i==2
        C{1} = cameraCapture1(C{1},W1);     % First camera only
    end
    C{i} = cameraCapture1(C{i},W1);         % i-th camera
    % (b) Matching (index of correspondence)
    [id_global,id_C1,id_C2] = getPixelCorrespondence(C{i-1},C{i},0);
    % (c) Essential Matrix
    E = getEssentialMatrix(C{i-1},C{i},id_C1,id_C2);
    % (d) Recover Motion (all cases)
    Rt_all = essentialMat2projectionMats(E);
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
    
    % >>>>>>>>>> Compute relative scale
    % http://rpg.ifi.uzh.ch/docs/Visual_Odometry_Tutorial.pdf
    if length(ind1)>=2      % distance needs at least two points
        X1 = Xt(:, ind1);
        X2 = X(:, ind2);
        % Find distance between all points in each set
        % All combinations without repetition of considering the same
        % point, i.e. consider the upper triangle on n*n matrix of indices
        % For speed we can use a few instead of all.
        ind_t0 = find(triu(reshape(1:length(ind1)^2, length(ind1), []),1));
        [ind_t1, ind_t2] = ind2sub(length(ind1), ind_t0);
        num = distance_point_point(X2(:,ind_t1), X2(:,ind_t2));
        den = distance_point_point(X1(:,ind_t1), X1(:,ind_t2));
        r_all = num./den;
        r = mean(r_all);
        %r = median(r_all);      % use this one in the presence of outliers

        % >>>>>>>>>>>>>>>>>>>>>>>
        % Change the above structure to differentiate between camera
        % pose and transformation between cameras.
        % Change the above <triangulate3DPoint1> function to return 3D
        % points wrt first camera, also return best transformation.
        % Also include lines as well as points.
        % >>>>>>>>>>>>>>>>>>>>>>>
        
        
    end
    
    
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
%     Plot cameras
cameraPlot1(C{1},'C_1')
hold on
for i=2:length(C)
    cameraPlot1(C{i},['C_' num2str(i)])
end
%     Plot Points
plotPoints(W1.point,'b.',10);                       % All points in <W1>
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
