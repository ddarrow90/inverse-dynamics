%given two vectors u, v, finds the projection of u onto v

function x = proj(u,v)
    if(numel(v) ~= 3 || numel(u) ~= 3)
        disp('proj error: both parameters need to be a 3d vector');
        x = NaN;
    return
    else
        x = dot(makevert(u),makevert(v))/dot(makevert(v),makevert(v))*makevert(v);
    end
end