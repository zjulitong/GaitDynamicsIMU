function [T] = Tmaker(o,i,j,k)

%Creates Transpose matrix at origin 'o' from an 3x1 rotation matrix

T1=[i,j,k,o];
T=[T1;[0 0 0 1]];
end

