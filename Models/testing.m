%testing area for various things, mainly attitude quaternion and keeping
%the "z" principal axis coplanar with segments

dummy1 = [1; 2]
dummy1(1)
dummy1(2)
dummy2 = [1, 2]
dummy2(1)
dummy2(2)
dummy3 = [0; dummy1]
ident = [[1;0;0],[0;1;0],[0;0;1]]
skewsymee = [[1;2;3],[1;2;3],[1;2;3]]
cross(skewsymee, ident)

makeskewsym([1,2,3])

%in = [1,2,3];
%unitin = makeunit(in)
%a = quatify(atand(in(2)/in(1)),[0,0,1])
%b = quatify(asind(in(3)/norm(in)),cross([in(1),in(2),0],in))
%c = quatmultiply(b, a)
%c2 = makeunit(c)

%theta = acosd(dot(in,[1,0,0])/norm(in))
%d = quatify(theta, cross([1,0,0],in))
%dinv = quatinv(d)
%shouldbe1 = quatmultiply(d, dinv)
%quatrotate(quatify(90,[1,0,0]),in)

%quatrotate(b,quatrotate(a, [1,0,0]))*norm(in)
%quatrotate(c, [1,0,0])*norm(in)
%quatrotate(d, [1,0,0])*norm(in)