function plotFiles(fileNames)

figure;
clf;
color = ["red", "blue", "green"];
numFiles = size(fileNames);
numFiles = numFiles(2);

for fileNum = 1:numFiles
    file = fileNames(fileNum)
    
    a = load(file);
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

    %scatter3(x,y,z,1,v);
    fv = isosurface (X,Y,Z,V,0.0);
    p = patch(fv);
    set(p,'FaceColor',color(1+mod(fileNum,2)),'EdgeColor','none');
    camlight;
    lighting gouraud;
    view(103, 37)
    
    axisSize = size(X)
    axisSize = axisSize(1)
    axis([0 axisSize 0 axisSize 0 axisSize]);
    xlabel('x')
    ylabel('y')
    zlabel('z')
end

end

