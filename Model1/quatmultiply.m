%poor man's ASTB quatmultiply
function x = quatmultiply(u, v)
    temp1 = [u(2), u(3), u(4)];
    temp2 = [v(2), v(3), v(4)];

    if(numel(v) ~= 4 || numel(u) ~= 4)
        disp("not two quaternions, cannot multiply");
    return
    else
        x = [u(1)*v(1)-dot(temp1,temp2),cross(temp1,temp2)+u(1)*temp2+v(1)*temp1];
end