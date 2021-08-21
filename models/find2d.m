%find func but instead it returns the coords [row index; column index] of
%the nonzero element in a 2d matrix
function x = find2d(mat)
    if(numel(size(mat)) ~= 2)
        disp("please input 2d matrix: failed for the matrix ");
        disp(mat);
        return
    end
    
    dimens = (size(mat));
    %height = dimens(1);
    %width = dimens(2);
    %ones = find(mat);
    %count = numel(ones);
    temp = zeros(2, numel(find(mat)));
    
    for i = 1:numel(find(mat))
        
    end
    
    x = temp;
end