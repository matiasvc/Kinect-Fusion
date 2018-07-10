function plotExtrinsic(A)

preloc = A(1:3, 4)
 rot = A(1:3, 1:3);
 det(rot)
 Ainv = inv(A);
 loc = Ainv(1:3, 4)
 loc = loc *100.0 / 2.0;
plotCamera('Location', loc, 'Orientation', rot, 'Size',2);


end