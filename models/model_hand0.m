g = -9.81;
thetainput = 60;
n = 3;

%hand orientation inputs (unit)
handx = [1;0;0];
handz = [0;0;1];
handy = cross(handz, handx);
if(dot(handx, handz) ~= 0)
    disp("hand axis vectors must be orthogonal");
    return
end

attquat = zeros(4,n);
attquat(:, 3) = attquatify(handx, handz);

%distance from distal end (farther end of fist) that holds

%n is number of segments -(including the ground "segment")-
%maybe change later so that we input the values (mass, theta, tensor
%princip axis values) from command prompt
m = zeros(n);
m = [4; 2; 1];
principax = zeros(3, n);
principax(:,3) = [1;0.7;0.5];
principax(:,2) = [6;2;1];
principax(:,1) = [4;2;1];
scstens = zeros(3,3,n);
icstens = zeros(3,3,n);

for i=1:n
    scstens(:,:,i) = [[principax(1,i);0;0],[0;principax(2,i);0],[0;0;principax(3,i)]];
end

%8/16 change: tree implementation
%have to manually input the directed graph
%are the indices of the segments)
segcons = [n + 1; 1; 2];
treehier = hierarchy(segcons);

%muscstuff
muscn = 2;
muscends = zeros(4,muscn);
muscends = [[0.9;1;2;1],[1.1;1;2;1]];

%experimenting with this value, trying to figure out how the muscle force exerted
%should be related to some directly inputed value (if the person decides
%to flex/relax their arm), the length the muscle is being stretched (some
%kind of tension), and the load on the muscle (since it can reduce/add
%stress to some spots)
muscmag = zeros(muscn);


%weight stuff
weightn = 1;
mdumb = zeros(weightn);
mdumb(1) = 8;
%contains the index + ratio from the proximal end of the weight
weightseg = zeros(2, weightn);
weightseg(:, 1) = [3; 0.75];
inweight = zeros(3, weightn);

for i = 1:weightn
    inweight(:, i) = [0;0;mdumb(i) * g];
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

com = [0.536, 0.4, 0.3];
leng = [1.62, 2, 0.5];

%we let the last distal vector be the origin, do not change it

distal = zeros(3, n + 1);

%{defining the location of dumbbell (x,y,z)%}

x = 1.4;
y = 1.5;
z = 2;

reach = [x; y; z];

%fist end pt
distal(:, 3) = reach + ((1 - weightseg(2, 1)) * handx * leng(3));
%wrist end pt
distal(:, 2) = reach - (weightseg(2, 1) * handx * leng(3));

%CHANGE: no longer right, the endpoint of the hand (dist1) is different
%from the point where the hand is holding the object
%distal([1,2,3],1) = reach;

%{checking triangle ineq in case I'm an idiot with test cases, prints error and ends early%}

if(norm(distal(:, 2)) > leng(1) + leng(2))
    disp('Get a longer arm');
    return
end

displace = LOC(leng(2), norm(distal(:, 2)), leng(1));

if(norm(distal(:, 2)) == leng(1) + leng(2))
    distal(:, 1) = (leng(1) / (leng(1) + leng(2))) * distal(:, 2);
elseif(abs(dot(makeunit(handx), makeunit(distal(:, 2)))) == 1 || abs(dot(makeunit(handx), makeunit(distal(:, 2)))) == 0)
    %it's physically impossible for the hand to be pting in opposite
    %directions as the shoulder to wrist, so we ignore that case (later on
    %we'll impose restraints on the range of motion of joints to guarantee
    %that this doesnt occur)
    
    %we choose the positioning of the elbow s.t. forearm x, z axes are
    %coplanar with those of the hand, preferring to minimize angle at wrist
    
    distal(:, 1) = distal(:, 2) + quatrotate(quatify(-displace, makeunit(cross(handz, handx))), (-makeunit(distal(:, 2)) * leng(2)));
else
    %here we can find the elbow position that minimizes bending at the
    %wrist, which is done by finding the elbow pt closest to the line
    %formed by extending handx
    circcent = distal(:, 2) + (makevert(-makeunit(distal(:, 2))) * cosd(displace) * leng(2));
    handdirint = line_plane_intersection(handx, distal(:, 2), distal(:, 2), circcent);
    distal(:, 1) = circcent + (leng(2) * sind(displace) * makevert(makeunit(makevert(handdirint) - circcent)));
end


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
    segvec(:, i) = distal(:, i) - distal(:, segcons(i));
    %jank fix for now, as we trnasition into using attquatify for all
    %segments
    %
    %
    %
    %making space so i remember to look here later
    if(i == 1 || i == 2)
        attquat(:, i) = makeunit(quatmultiply(quatify(asind(segvec(3, i)/norm(segvec(:,i))),cross([segvec(1,i),segvec(2,i),0],segvec(:,i))), quatify(atand(segvec(2,i)/segvec(1,i)),[0,0,1])));
    end
    
    attmat(:,:,i) = aqtoam(attquat(:, i));
    %testing
    %disp(quatmultiply(quatify(asind(segvec(3, i)/norm(segvec(:,i))),cross([segvec(1,i),segvec(2,i),0],segvec(:,i))), quatify(atand(segvec(2,i)/segvec(1,i)),[0,0,1])));
    %disp(attmat(:,:,i));
    %disp(i);
    %disp(distal(:,i));
    %q: check just for myself that attitude matrix is always invertible
    %(just thinking conceptually, it should be)
    icstens(:,:,i) = mtimes(attmat(:,:,i),mtimes(scstens(:,:,i),inv(attmat(:,:,i))));
end

%everything is set up, solve equations here:
%remember that because it's static, we don't care about the dynamic wrench
%and it has been omitted???it will be added in later models
%rather than treating the ground/center of pressure as another wrench, I
%treat it as just a force and moment as for this model it's more convenient
%(considering it a wrench overcomplicated things and brought about more
%issues than fixes)

%n+1 wrenches, n distal ends + origin
dwrenches = zeros(6,n+1);

%for all segments of highest hierarchy index, we must set their wrenches;
%since the weight being held is now along the hand instead of at its exact
%endpoint, we can ignore it and make the weightwrenches instead
%dwrenches(:,3) = [inweight;makevert([0,0,0])];
weightwrenches = zeros(6, n+1);
for i = 1:n+1
    found = find(weightseg(1, :) == i);
    for j = 1:numel(found)
        weightwrenches(:, treehier(weightseg(1, found(j)))) = weightwrenches(:, treehier(weightseg(1, found(j)))) + vertcat(inweight(:, found(j)), cross(segvec(:, weightseg(1, found(j))) * weightseg(2, found(j)), inweight(:, found(j))));
    end
end

nothing = [[0,0,0];[0,0,0];[0,0,0]];
id = [[1,0,0];[0,1,0];[0,0,1]];
muscwren = zeros(6,n+1);
%this iteration adds a muscle so we add a new wrench, need to find an
%easy way to keep track of which muscles are relevant to which segments
%since their index doesnt say much

muscvecs = zeros(3,muscn);
%setting direction of muscvecs first, making unit norm 1
for i=1:muscn
    muscvecs(:,i) = (1-muscends(1,i))*segvec(:,muscends(3,i))+muscends(2,i)*segvec(:,muscends(4,i));
end

disp('pre-musc wrenches:');


for i = max(treehier):-1:1
    %instead of making vectors beforehand for the muscles' orientation, we
    %can just add up the segment vectors multiplied by the appropriate
    %ratios, then multiply by the magnitude of the force (which can be
    %input after the results of the final wrench are found symbolically)
    found = find(treehier == i);
    for j = 1:numel(found)
        arr1 = vertcat(horzcat(m(treehier(found(j)))*id, nothing),horzcat(m(treehier(found(j)))*makeskewsym(com(treehier(found(j)))*segvec(:, treehier(found(j)))), icstens(:, :, treehier(found(j)))));
        arr2 = vertcat(horzcat(id, nothing),horzcat(makeskewsym(segvec(:, treehier(found(j)))),id));
        dwrenches(:, segcons(treehier(found(j)))) = arr1*[0;0;g;0;0;0]+arr2*dwrenches(:, treehier(found(j))) + weightwrenches(:, segcons(treehier(found(j))));
        
        
        disp(dwrenches(:, segcons(treehier(found(j)))));
    end
end

%moment in the direction of the muscles
muscf = -proj(dwrenches(4:6,1),cross(muscvecs(:,2),muscvecs(:,1)));
%checks which way the muscle moment points---if in the same direction as cross prod then bicep 
if(dot(muscf,cross(muscvecs(:,2),muscvecs(:,1))) > 0)
    muscmag(2) = norm(muscf)/norm(cross(segvec(:,muscends(3,2))*(1-muscends(1,2)),muscvecs(:,2)));
else
    muscmag(1) = norm(muscf)/norm(cross(segvec(:,muscends(3,1))*(1-muscends(1,1)),muscvecs(:,1)));
end

%note that this only works for non-biarticular muscles that act between
%segs 1,2 (i.e. the elbow is between them)
%index of muscwren refers to the segment index it acts on
for j = 1:muscn
    muscwren(:,segcons(muscends(3,j))) = muscwren(:,segcons(muscends(3,j))) + vertcat(muscvecs(:,j)*muscmag(j), cross(segvec(:,muscends(3,j))*(1-muscends(1,j)),muscvecs(:,j)*muscmag(j)));
end
    
%muscwren(:,1) = vertcat(muscvecs(:,1),cross(muscvecs(:,1),segvec(:,1)*(1-muscends(1,1))));
%muscwren(:,2) = vertcat(-muscvecs(:,1),cross(-muscvecs(:,1),segvec(:,2)*(1-muscends(2,1))));

for i = max(treehier):-1:1
    %instead of making vectors beforehand for the muscles' orientation, we
    %can just add up the segment vectors multiplied by the appropriate
    %ratios, then multiply by the magnitude of the force (which can be
    %input after the results of the final wrench are found symbolically)
    found = find(treehier == i);
    for j = 1:numel(found)
        arr1 = vertcat(horzcat(m(treehier(found(j)))*id, nothing),horzcat(m(treehier(found(j)))*makeskewsym(com(treehier(found(j)))*segvec(:, treehier(found(j)))),icstens(:, :, treehier(found(j)))));
        arr2 = vertcat(horzcat(id, nothing),horzcat(makeskewsym(segvec(:, treehier(found(j)))),id));
        dwrenches(:, segcons(treehier(found(j)))) = arr1*[0;0;g;0;0;0]+arr2*dwrenches(:, treehier(found(j))) + weightwrenches(:, segcons(treehier(found(j)))) + muscwren(:, segcons(treehier(found(j))));
    end
end

%%%%%%%%%%%%%
%OUTPUT AREA%
%%%%%%%%%%%%%
disp("joint wrenches:");

for i=1:n+1
    disp(dwrenches(1:6,i));
end
%disp(dwrenches(:,n+1));

disp("muscle load vector:");
disp(muscf);
disp("muscle load:");
disp(norm(muscf));
disp("muscle used:");
if(dot(muscf,cross(muscvecs(:,2),muscvecs(:,1))) > 0)
    disp("bicep");
else
    disp("tricep");
end

%graphing (static) 
quiver3(0,0,0,0,0,0,'c');
hold on
%quiver3(0,0,0,-muscvecs(1,1)/10,-muscvecs(2,1)/10,-muscvecs(3,1)/10,'r');
for i=1:n+1
    %color key:
    %???red: forces
    %-blue: moments
    %-green: segments
    %trying out scaling down the wrenches by a factor of 10 to fit them
    %better, makes the segments easier to see
    quiver3(distal(1,i),distal(2,i),distal(3,i),dwrenches(1,i)/40,dwrenches(2,i)/40,dwrenches(3,i)/40,'r');
    %axis equal
    quiver3(distal(1,i),distal(2,i),distal(3,i),dwrenches(4,i)/20,dwrenches(5,i)/20,dwrenches(6,i)/20,'b');
    
    %plotting each of the arm segments as well
    if(i < n+1)
        t =  0:1/100:1;
        plot3(distal(1,i)-distal(1,i)*t+distal(1,segcons(i))*t,distal(2,i)-distal(2,i)*t+distal(2,segcons(i))*t,distal(3,i)-distal(3,i)*t+distal(3,segcons(i))*t,'g')
    end
    %axis([0 10 0 10 0 100])
end