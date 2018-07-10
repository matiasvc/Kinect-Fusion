function scatterFiles(fileNames, camFile)

figure;
clf;
hold on;
numFiles = size(fileNames);
numFiles = numFiles(2);

C = load(camFile);
lines = size(C);
for i = 4:4:lines
    c = C(i-3:i, :);
    plotExtrinsic(c);
end


for fileNum = 1:numFiles
    file = fileNames(fileNum)
    
    a = load(file);
    xa = a(:, 1);
    ya = a(:, 2);
    za = a(:, 3);
    va = a(:, 4);
    
    x = [];
    y = [];
    z = [];
    v = [];
    
    for i = 1:size(xa)
       if va(i) ~= 1
            x = [x xa(i)];
            y = [y ya(i)];
            z = [z za(i)];
            v = [v va(i)];
       end
    end


    scatter3(x,y,z,1,v);

    view(90, 30)
    
    axisSize = max(xa);
    axis([0 axisSize 0 axisSize 0 axisSize]);
    xlabel('x')
    ylabel('y')
    zlabel('z')
end

end

