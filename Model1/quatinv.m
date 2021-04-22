%poor man's quatinv
function x = quatinv(q)
    if(numel(q) ~= 4)
        disp("please input a 1x4 quaternion and a 1x3 vector to rotate by quaternion");
    return
    else
        conj = [q(1), -q(2), -q(3), -q(4)];
        x = conj/(norm(q)^2);
end