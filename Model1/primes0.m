%CHANGELOG
%
%4/16/21
%this changelog was born
%reversed order of some stuff to reflect order it should be solved in
%removed ground segment
%
%4/17/21
%changed quatify to take (angle, axis) where axis is now a vector instead of
%its components, fixed it to halve angle (no need to divide by 2 before
%inputting angle)
%
%4/18/21
%fixed up some stuff related to rotating about DEG after getting the stuff
%in positiion relative to other segments, realized I could just rotate
%include the deg rotation in initrotaxis first to have the same effect
%made attitude quaternion stuff
%
%changes needed:
%describe better the "base" position of the z axis in the SCS before adding
%degs of freedom for the principal axes of inertia tensor
%
%for aqtoam, need to make it so that it automatically converts the input
%quaternion to vertical to not confuse things with qv*transpost(qv)
%

g = 9.81;
thetainput = 0;

%n is number of segments -(including the ground "segment")-
%4/16/21 change: no longer using ground segment, since at this point i
%considered it a point mass anyway
%maybe change later so that we input the values (mass, theta, tensor
%princip axis values) from command prompt
n = 2;
m = zeros(n);
m = [2; 4];
principax = zeros(3, n);
principax(:,1) = [6;2;1];
principax(:,2) = [4;2;1];
scstens = zeros(3,3,n);
icstens = zeros(3,3,n);

%for only this model
mdumb = 8;
inweight = [0;0;mdumb*g];

for i=1:n
    scstens(:,:,i) = [[principax(1,i);0;0],[0;principax(2,i);0],[0;0;principax(3,i)]];
end

%- treating the dumbbell (considered a pt mass for now) a.k.a. segment 0 as
%- having 0 length and 0 distance between 
%- CoM and P_0, later maybe change this to include the wrist as another joint and
%- hand as segment, which would be more accurate with the assumption of 0 length for dumbbell
%Note: D(i) is location of D_i, NOT the vector describing segment i
%- We assume that P(n)=0,0,0
% P(n), which we lose when changing to distal points, becomes the origin
% (unless we need to move it, in which case some new stuff needs to be
% added)

com = [0.4, 0.536];
leng = [2, 1.62];

%we let the last distal vector be the origin, do not change it

distal = zeros(3, n+1);

%{defining the location of dumbbell (x,y,z)%}

x = 1;
y = 1.1;
z = 0.9;

reach = [x,y,z];

distal([1,2,3],1) = reach;

%{checking triangle ineq in case I'm an idiot with test cases, prints error and ends early%}

if(norm(reach)>leng(1)+leng(2))
    disp('Get a longer arm')
    return
end

%quaternion corresponding to rotation of angle theta about <x,y,z>
%using degrees here, the default 0 degree rotation position will have x,y
%coordinates of all proximal pts collinear and the elbow (P1) below line
%000 to xyz, essentially completely vertical
%quatify takes in angle and axis, returns corresponding quaternion
%describing the rotation

%DEG = [cos(thetainput), sin(thetainput)*[x, y, z]/norm([x, y, z])]
DEG = quatify(thetainput, reach);

%law of cosines for angles to get relevant quaternions, to set up the
%vectors corresponding to the segments we use the quaternion with the
%calculated angles and axis set to the cross product of 001 and xyz

%LOC.m takes three side lengths, the first of which is the side opposite
%the desired angle, outputs in degrees

%thetaseg(i) is the angle between segment i-1 and i-2, where segment 0 is
%considered the line between the shoulder and distal end of arm (at least
%for the 0 length dumbbell case)

%Note (4/18/21): can't solve for thetas in higher cases like this, have to
%leave it in terms of other variables since there will be too many degrees
%of freedom

thetaseg = zeros(n - 1);
%acosd((-norm([x,y,z])^2-d(2)^2+d(3)^2)/(-2*norm([x,y,z])*d(2)));
%thetasegs(1) = LOC(d(2),norm([x,y,z]),d(1));
%thetasegs(2) = LOC
%thetasegs(2) = -(180 - acosd((norm([x,y,z])^2-d(2)^2-d(3)^2)/(-2*d(3)*d(2))));
thetaseg(1) = LOC(leng(2),leng(1),norm(reach));

%{creating quaternions, calculating attitude matrices for each segment (which are for now static)%}
%note that the initrot vector may change when there are more than 2
%segments, since right now we have the all segments are planar and thus are
%rotated about the same axis

initrotaxis = quatrotate(DEG ,[cross([0,0,1],[x,y,z])/norm([x,y,z])]);

for i = 1:n-1
    distal(:, i+1) = distal(:,i)+leng(i)*quatrotate(quatify(180+thetaseg(i),initrotaxis),distal(:, i)/norm(distal(:, i)));
end

%finding attitude quaternion
%for now I'm assuming that the segments have their principal axis in the
%"z" direction in their SCS in the same plane as the proximal points
%I feel like to generalize the method more I should somehow apply the
%atquat stuff to x,y,z though I guess that since I'm modeling the dumbbell
%as a point mass it won't be applicable yet

attquat = zeros(4,n);

%calculating attitude quaternions such that from the ICS to SCS(i):
%1) the x-axis goes along the length of the segment 
%2) the proximal end of the segment as the origin
%3) thw z-axis is coplanar with the two segments, "above" the segment
%(define "above" a bit better later)
%note: with more than 2 segments condition 3 will need to be modified

%in loop im also calculating attitude matrices from attitude quaternions
%aqtoam is a function that converts an attitude quaternion to matrix (see
%equation 24, R. Dumas)

attmat = zeros(3,3,n);
segvec = zeros(3,n);

for i=1:n
    segvec(:, i) = distal(:, i) - distal(:, i+1);
    attquat(:, i) = makeunit(quatmultiply(quatify(asind(segvec(3, i)/norm(segvec(:,i))),cross([segvec(1,i),segvec(2,i),0],segvec(:,i))), quatify(atand(segvec(2,i)/segvec(1,i)),[0,0,1])));
    attmat(:,:,i) = aqtoam(attquat(:, i));
    %q: check just for myself that attitude matrix is always invertible
    %(just thinking conceptually, it should be)
    icstens(:,:,i) = mtimes(attmat(:,:,i),mtimes(scstens(:,:,i),inv(attmat(:,:,i))));
end

%everything is set up, solve equations here:
%remember that because it's static, we don't care about the dynamic wrench
%and it has been omitted—it will be added in later models
%rather than treating the ground/center of pressure as another wrench, I
%treat it as just a force and moment as for this model it's more convenient
%(considering it a wrench overcomplicated things and brought about more
%issues than fixes)

%n+1 wrenches, n distal ends + origin
dwrenches = zeros(6,n+1);
dwrenches(:,1) = [inweight;norm(cross(segvec(:,1),inweight))*(inweight-(dot(segvec(:,1),inweight)*segvec(:,1))/(norm(segvec(:,1)))^2)];
nothing = [[0,0,0];[0,0,0];[0,0,0]];
id = [[1,0,0];[0,1,0];[0,0,1]];
for i=1:n
    arr1 = vertcat(horzcat(m(i)*id, nothing),horzcat(m(i)*makeskewsym(com(i)*segvec(:,i)),icstens(:,:,i)));
    arr2 = vertcat(horzcat(id, nothing),horzcat(makeskewsym(segvec(:,i)),id));
    dwrenches(:,i+1) = mtimes(arr1,[0;0;-g;0;0;0])+mtimes(arr2,dwrenches(:,i));
end
disp(dwrenches(:,n+1));