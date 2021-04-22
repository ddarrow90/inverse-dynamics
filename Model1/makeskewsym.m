%makes a size 3 column vector into skew sym form
function x = makeskewsym(v)
    if(numel(v) ~= 3)
        disp('v needs to be size 3');
        x = NaN;
        return
    end
    x = cross([makevert(v), makevert(v), makevert(v)], [[1;0;0],[0;1;0],[0;0;1]]);
end