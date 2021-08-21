%given x axis and z axis in the SCS in the form of vectors, finds the
%attitude quaternion that maps 100 and 001 to this x, z axis
function out = attquatify(xin, zin)
    if(numel(xin) ~= 3 || numel(zin) ~= 3 || dot(xin,zin) ~= 0)
        disp("Not two orthogonal vectors. attquatify failed to execute for these two inputs:");
        disp(xin);
        disp(zin);
        return
    else
        %these two planes are formed by the possible axes that you could
        %use to rotate the x axis in ics to x axis in scs, same for
        %z---the intersection of these two planes, if it's a line, should
        %be the axis for the attitude quaternion
        
        %we have two issues:
            %1) if either or both of the x,z axes in the ics and scs are the
            %exact same, then there's no planex or planez
            %2) if the ICS-SCS x,z pairs of axes are reflections of each
            %other across the same plane then planez and planez are the
            %same (the other issue stopping us from having a line
            %intersection would be if the planes were parallel, but we know
            %that 0,0,0 is a common point so we don't need to consider this
            %case
        x = makeunit(xin);
        z = makeunit(zin);
            
        if(isequal(makevert(x),[1;0;0]) || isequal(makevert(x),[-1;0;0]))
            %in this case we just use the x axis as our axis of rotation
            if(isequal(makevert(z),[0;1;0]) || isequal(makevert(z), [0;-1;0]))
                %division by 0 in atand(-z(2)/z(3)) part
                out = makeunit(quatmultiply(quatify(180*((1-x(1))/2),[0;1;0]),quatify(90*(-z(2)),[1;0;0])));
            elseif(isequal(makevert(z), [0;0;1]) || isequal(makevert(z), [0;0;-1]))
                %division by 0 in (z(2)/abs(z(2))) part
                out = makeunit(quatmultiply(quatify(180*((1-x(1))/2),[0;0;1]),quatify(180*((1-z(3))/2),[1;0;0])));
            else
                out = makeunit(quatmultiply(quatify(atand(-z(2)/z(3)) + 180*(((z(2)/abs(z(2)))+1)/2),[1;0;0]), quatify(180*((1-x(1))/2),[0;0;1])));
            end
        elseif(isequal(makevert(z), [0;0;1]) || isequal(makevert(z), [0;0;-1]))
            %similarly, we use z axis
            %we don't need an if statement block here because in the main
            %output part, only x(2) is used as a denominator anywhere, and
            %fails only when x(2) = 0---but because of orthogonality, we must
            %have that x(2) = 0 implies x(1) = +/-1, which is already
            %covered in the previous section
            out = makeunit(quatmultiply(quatify(atand(x(1)/x(2)) + 180*((1-(x(2)/abs(x(2))))/2), [0;0;1]), quatify(180*((1-z(3))/2),[1;0;0])));
        else
            planex = cross((makevert(x)+[1;0;0])/2,cross(makevert(x),[1;0;0]));
            planez = cross((makevert(z)+[0;0;1])/2,cross(makevert(z),[0;0;1]));
            if(proj(planex, planez) == planex)
                %if the x,z pairs are refs of each other across same plane
                out = quatify();
            else
                %if the intersection exists as a line
                out = [1;0;0;0];
            end
        end
    end
end