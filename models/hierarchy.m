%given a permutation matrix describing edges from i to in(i)
function out = hierarchy(matin)
    sizemat = size(matin);
    
    if(numel(sizemat) ~= 2)
        disp("needs vector, too many dimensions (>3)");
        return;
    elseif(sizemat(1) > 1 && sizemat(2) > 1)
        disp("needs vector, too many dimensions (2)");
        return;
    end

    %these are the connections btwn segments
    %the actual initial codes for code
    %segcons(1, :) = [0, 1, 2];
    
    n = numel(matin);
    segcons = zeros(2, n);
    %the (unique) segment connecting to 0 is the root
    %must be a tree (too lazy to write something that'll check for loops in
    %graph)
    segcons(1, :) = transpose(makevert(matin));
    segcons(2, 1) = 1;
    whitelist = zeros(n - 1);
    whitelist(1) = 1;
    whitelistnext = zeros(n - 1);
    wlncount = 1;
    segchecked = 1;
    currentlevel = 2;

    while(segchecked < n)
        for i = 1:numel(find(whitelist))
            temp = find(segcons(1, :) == whitelist(i));
            for j = 1:numel(temp)
                whitelistnext(wlncount) = temp(j);
                wlncount = wlncount + 1;
                segcons(2, temp(j)) = currentlevel;
                segchecked = segchecked + 1;
            end
        end
        wlncount = 1;
        whitelist = whitelistnext;
        whitelistnext = zeros(n - 1);
        currentlevel = currentlevel + 1;
    end
    
    out = segcons(2, :);
end
