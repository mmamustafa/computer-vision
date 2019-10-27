function w = rotationMatrix2Vector(R)
% -------------------------------------------------------------------------
% This function converts rotation from Lie group "SO(3)" to Lie algebra
% "so(3)".
%
% Input:
%   <R>     (3,3)   Rotation matrix in SO(3).
%
% Output:
%   <w>     (3,1)   Rotation vector in so(3). Angle-axis form.
%
% Implementation:   Mohamed Mustafa
%                   University of Manchester, July 2017
% Modifications:    ...
%
% References:
%   - https://en.wikipedia.org/wiki/Axis?angle_representation#Log_map_from_SO.283.29_to_so.283.29
%   - https://en.wikipedia.org/wiki/Axis?angle_representation#Exponential_map_from_so.283.29_to_SO.283.29
%   - https://www.youtube.com/watch?v=khLM8VV8LuM&list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4&index=3
% -------------------------------------------------------------------------

% % (1) Implementation for SO(3)
% %     Has problems when theta=pi or theta=-pi   (fix later...)
% theta = reduce_angle_fullCircle(acos((trace(R)-1)/2));      % [-pi:pi]
% if isEqual(theta,0)
%     w = [0 0 0]';
% elseif isEqual(theta,pi) || isEqual(theta,-pi)
%     % >>> Do later... <<<
% else
%     w = theta*[R(3,2)-R(2,3)  R(1,3)-R(3,1)   R(2,1)-R(1,2)]'/(2*sin(theta));
% end

% (1) >>>> Another Implementation <<<<<
w = skewSymMat2vec(logm(R));
return