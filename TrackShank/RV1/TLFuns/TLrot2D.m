function R=TLrot2D(theta,flag)
% This function is calculate the rotation matrix from a given angle in 2D

% Author: Tong Li
% Date: Oct. 8, 2019;

%% INPUT:
% theta: the angle of rotation, the unit must be radian if in double
% format. symbolic, double
%% OUTPUT:
% R: the rotation matrix.

Rmat=[cos(theta), -sin(theta);
      sin(theta),  cos(theta)];

switch flag
    case 1
        R=Rmat(1,:);
    case 2
        R=Rmat(2,:);
    case 0
        R=Rmat(:,:);
end


end
