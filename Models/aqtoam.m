%given attitude quaternion, calculates attitude matrix from equation 24 of
%R. Dumas

function x = aqtoam(q)
    if(numel(q) ~= 4 || isequal(size(q), [2, 2]))
        disp('needs quaternion as input');
        x = NaN;
        return
    end
    t = makevert(q);
    %the dot product should be the same thing as whatever they did in
    %eq 24
    qv = [t(2); t(3); t(4)];
    x = zeros(3,3)*dot(t,t) + 2*mtimes(qv, transpose(qv))+2*t(1)*makeskewsym(qv);
end