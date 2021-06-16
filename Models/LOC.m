%function to solve law of cosines, in the input form (side opposite desired
%angle, either of the other sides, last side)

%output in degrees btwn 0-180

function x = LOC(d1,d2,d3)

x = acosd((-d2^2-d3^2+d1^2)/(-2*d2*d3));

end