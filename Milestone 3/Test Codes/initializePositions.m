function [points] = initializePositions(r,n)

radiusofBigCircle = (pi*r)/(n-pi);
centerofCircle = ceil(radiusofBigCircle + 2*r);
separationAngle = floor(360/n);
points = [];
for i = 1:n
    pointonCircle = [2*r+radiusofBigCircle+r*cosd((i-1)*separationAngle) 2*r+radiusofBigCircle+r*sind((i-1)*separationAngle)];
    points = [points;pointonCircle];
end

end