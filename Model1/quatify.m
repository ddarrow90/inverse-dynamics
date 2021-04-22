%given angle and axis (in form of vector), returns unit quaternion

function x = quatify(theta,v)
    if(numel(v) ~= 3 || numel(theta) ~= 1)
        disp('first parameter is angle in degrees, second parameter needs to be a 3d vector');
        x = NaN;
    return
    else
        x = [cosd(theta/2),(sind(theta/2))*v(1)/norm(v),(sind(theta/2))*v(2)/norm(v),(sind(theta/2))*v(3)/norm(v)];
    end
end