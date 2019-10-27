% >>>>>> Test Rotation representations and transformations <<<<<<<<

clc, clear all

% (1) generate rotations in so(3)
w1 = uniSampleND(repmat(5*[-1 1],1,3));

% (2) put <w> in miminum angle represntation [-pi:pi]
w1 = rotationVectorNormalized(w1);

% (3) so(3) --> SO(3)
R1 = rotationVector2Matrix(w1);

% (4) SO(3) --> so(3)
w1_ = rotationMatrix2Vector(R1);

disp('Columns should be the same:')
[w1 w1_]


% >>>>>> Test composition of Rotation forms <<<<<<<<<<<<<<<<<<<<<<<<
% >>>>>> Approximation Works if the rotation is small <<<<<<<<<<<<<<
% >>>>>> Multiple View Geometry (pages: 624) <<<<<<<<<<<<<<<<<<<<<<<

% (1) Rotations in angle-axis form:	so(3)
w1 = pi/2*[1 1 0]';
w2 = pi/2*[1.1 1.2 0]';

% (2) Rotations in matrix form:     SO(3)
R1 = rotationVector2Matrix(w1);
R2 = rotationVector2Matrix(w2);

% (3) add angle-axis rotations
w3 = rotationVectorNormalized(w1+w2);

% (4) Multiply matrix rotations
R3 = R1*R2;

% (5) Check equivalency
w3_ = rotationVectorNormalized(rotationMatrix2Vector(R3));
R3_ = rotationVector2Matrix(w3);

% (6) Display everything
w3, w3_
R3, R3_















