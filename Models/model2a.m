%adds bicep
%treats the bicep as a force with known input, we can play around with
%values to see what force should minimize stress on each joint

g = -9.81;
thetainput = 60;

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

%angles for the vertices adjacent to the endpoints of the muscles in the
%triangle formed by the joint between endpts of muscles and said endpts,
%first value is the "distal end" second is "proximal"
%muscle index key:
%1 = bicep
musctheta = zeros(2,1);

%locations of endpts of muscles
%first value is the "distal end" second is "proximal"
%third/fourth values are the segment numbers referred to by values 1 and 2,
%respectively
muscends = [0.9;1;1;2];

%experimenting with this value, trying to figure out how the muscle force exerted
%should be related to some directly inputed value (if the person decides
%to flex/relax their arm), the length the muscle is being stretched (some
%kind of tension), and the load on the muscle (since it can reduce/add
%stress to some spots)
muscf = 10;
%syms muscf;

%for only this model
mdumb = 3;
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

%find musctheta (convert to for loop later), note that biarticular muscles
%cant use this due to a) no triangle b) screwed up indices for angle and
%segment stuff
pmusctemp = (leng(muscends(3,1))*(1-muscends(1,1)));
dmusctemp = (leng(muscends(4,1))*(muscends(2,1)));
musclengtemp = sqrt(pmusctemp^2+dmusctemp^2-2*dmusctemp*pmusctemp*cosd(thetaseg(1)));
musctheta(1,1) = LOC(pmusctemp, dmusctemp, musclengtemp);
musctheta(2,1) = 180 - musctheta(1,1) - thetaseg(1);

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
%dwrenches(:,1) = [inweight;makevert(cross(segvec(:,1),inweight))];
%interpreted equation wrong, the input weight does not also become a
%moment, wrench1 moment would be something like actually an input torque
%directly twisting the hand
dwrenches(:,1) = [inweight;makevert([0,0,0])];
nothing = [[0,0,0];[0,0,0];[0,0,0]];
id = [[1,0,0];[0,1,0];[0,0,1]];
muscwren = zeros(6,n);
%this iteration adds a muscle so we add a new wrench, need to find an
%easy way to keep track of which muscles are relevant to which segments
%since their index doesnt say much
muscvecs = zeros(3,1);
muscvecs(:,1) = makeunit((1-muscends(1,1))*segvec(:,muscends(3,1))+muscends(2,1)*segvec(:,muscends(4,1)))*muscf;
muscwren(:,1) = vertcat(muscvecs(:,1),cross(muscvecs(:,1),segvec(:,1)*(1-muscends(1,1))));
muscwren(:,2) = vertcat(-muscvecs(:,1),cross(-muscvecs(:,1),segvec(:,2)*(1-muscends(2,1))));
for i=1:n
    %instead of making vectors beforehand for the muscles' orientation, we
    %can just add up the segment vectors multiplied by the appropriate
    %ratios, then multiply by the magnitude of the force (which can be
    %input after the results of the final wrench are found symbolically)
    arr1 = vertcat(horzcat(m(i)*id, nothing),horzcat(m(i)*makeskewsym(com(i)*segvec(:,i)),icstens(:,:,i)));
    arr2 = vertcat(horzcat(id, nothing),horzcat(makeskewsym(segvec(:,i)),id));
    dwrenches(:,i+1) = arr1*[0;0;g;0;0;0]+arr2*dwrenches(:,i)+muscwren(:,i);
end

%%%%%%%%%%%%%
%OUTPUT AREA%
%%%%%%%%%%%%%

for i=1:n+1
    disp(dwrenches(1:6,i));
end
%disp(dwrenches(:,n+1));

%graphing (static) 
quiver3(0,0,0,muscvecs(1,1)/10,muscvecs(2,1)/10,muscvecs(3,1)/10,'r');
hold on
%quiver3(0,0,0,-muscvecs(1,1)/10,-muscvecs(2,1)/10,-muscvecs(3,1)/10,'r');
for i=1:n+1
    %color key:
    %—red: forces
    %-blue: moments
    %-green: segments
    %trying out scaling down the wrenches by a factor of 10 to fit them
    %better, makes the segments easier to see
    quiver3(distal(1,i),distal(2,i),distal(3,i),dwrenches(1,i)/10,dwrenches(2,i)/10,dwrenches(3,i)/10,'r');
    %axis equal
    quiver3(distal(1,i),distal(2,i),distal(3,i),dwrenches(4,i)/10,dwrenches(5,i)/10,dwrenches(6,i)/10,'b');
    
    %plotting each of the arm segments as well
    if(i < n+1)
        t =  0:1/100:1;
        plot3(distal(1,i)-distal(1,i)*t+distal(1,i+1)*t,distal(2,i)-distal(2,i)*t+distal(2,i+1)*t,distal(3,i)-distal(3,i)*t+distal(3,i+1)*t,'g')
    end
    %axis([0 10 0 10 0 100])
end



%attempt at graphing with animation, iterating along thetainput