%poor man's quatrotate

function x = quatrotate(q, v)

    if(numel(q) ~= 4 || numel(v) ~= 3 || isequal(size(q), [2, 2]))
        disp("please input a 1x4 quaternion and a 1x3 vector to rotate by quaternion");
        x = NaN;
    return
    else
        temp = quatmultiply(quatmultiply(q,[0; makevert(v)]),quatinv(q));
        x = [temp(2); temp(3); temp(4)];
    end
end