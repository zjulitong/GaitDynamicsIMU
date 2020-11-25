function theta=TL_Xangle(vec,flag)


y=vec(2);
z=vec(3);

switch flag % axis vector to use
    case 'k'
        theta=atan(-y/z); % w.r.t. Z axis, countclock positive
    case 'j'
        theta=atan(z/y); % w.r.t. Y axis, countclock positive
end

% theta=rad2deg(theta);


end
