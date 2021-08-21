%given x axis and z axis in the SCS in the form of vectors, finds the
%attitude quaternion that maps 100 and 001 to this x, z axis
function out = attquatify(xin, zin)
    if(numel(xin) ~= 3 || numel(zin) ~= 3 || dot(xin,zin) ~= 0)
        disp("Not two orthogonal vectors. attquatify failed to execute for these two inputs:");
        disp(xin);
        disp(zin);
        return
    else
        x = makevert(makeunit(xin));
        z = makevert(makeunit(zin));
        
        planeend = cross(x, z);
        
        quat1 = [1;0;0;0];
        quat2 = [1;0;0;0];
        quat3 = [1;0;0;0];
        xtracker = [1;0;0];
        ztracker = [0;0;1];
        
        if(abs(dot([0;1;0], planeend)) ~= 1)
            quat1 = makeunit(vertcat(1 + dot([0;1;0], planeend), cross([0;1;0], planeend)));
            xtracker = quatrotate(quat1, xtracker);
            ztracker = quatrotate(quat1, ztracker);
        end
        
        if(abs(dot(xtracker, x)) ~= 1)
            quat2 = makeunit(vertcat(1 + dot(xtracker, x), cross(xtracker, x)));
        else
            quat2 = quatify(180*((1-dot(xtracker, x))/2), planeend);
        end
        
        ztracker = quatrotate(quat2, ztracker);
        
        quat3 = quatify(180*((1-dot(ztracker, z))/2), x);
        
        out = quatmultiply(quat3, quatmultiply(quat2, quat1));
    end 
end