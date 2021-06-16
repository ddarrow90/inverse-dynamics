%makes a vector vertical
function x = makevert(v)
    if(size(v,1) == 1)
        x = transpose(v);
    elseif(size(v,2) == 1)
        x = v;
    else
        x = NaN;
        disp("v must be vector");
    end
end