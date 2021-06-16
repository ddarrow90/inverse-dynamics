%given any vector, makes it unit norm

function x = makeunit(v)
    x = zeros(1, numel(v));

    tot = 0;
    
    for i = 1:numel(v)
        tot = tot + v(i)^2;
    end
    
    for i = 1:numel(v)
        x(1, i) = v(i)/sqrt(tot);
    end
end