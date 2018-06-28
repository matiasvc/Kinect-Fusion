a = load('cmake-build-debug/TSDF.csv');
x = a(:, 1);
y = a(:, 2);
z = a(:, 3);
v = a(:, 4);



[X, Y, Z] = meshgrid(unique(x), unique(y), unique(z)); 
V = ones(size(X));

for i = 1:size(v)
    xi = x(i) +1;
    yi = y(i) +1;
    zi = z(i) +1;
    val = v(i);
  V(xi, yi, zi) = val;
end

size(X)
size(V)

figure;
clf;
%scatter3(x,y,z,1,v);
fv = isosurface (X,Y,Z,V,0.0);
p = patch(fv);
set(p,'FaceColor','red','EdgeColor','none');
camlight;
lighting gouraud;
view(103, 37)

axis([0 70 0 70 0 70]);
xlabel('x')
ylabel('y')
zlabel('z')