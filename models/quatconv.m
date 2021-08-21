%given two directions described by vectors u, v, finds quaternion 
%transforming unit vector in the direction of u to v
%generally i don't like using this function alone, as it leaves the
%orientation of the other 2 axes untracked—you can figure it out pretty easily,
%though, since it does the shortest arc rotation
%DO NOT USE FOR PARALLEL/OPPOSITE DIRECTION VECTORS, I DONT CHECK IN THE
%CODE
function out = quatconv(uin, vin)
    if(numel(uin) ~= 3 || numel(vin) ~= 3)
        disp("must be two vectors");
        disp(uin);
        disp(vin);
        return
    else
        u = makevert(makeunit(uin));
        v = makevert(makeunit(vin));
    end
    if(abs(dot(u, v)) == 1)
        out = NaN;
    else
        out = makeunit(vertcat(1 + dot(u, v), cross(u, v)));
    end 
end