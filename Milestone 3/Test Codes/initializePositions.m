%This function gives the points of the initial position
%of each robot, it takes (track width*2 of each robot,number of robots) as inputs
function [points] = initializePositions(r,n)

radiusofBigCircle = (pi*r)/max((n-pi),1); %????
%centerofCircle = ceil(radiusofBigCircle + 2*r); %????
separationAngle = floor(360/n);
points = []; %Output Points
for i = 1:n
    pointonCircle = [2*r+radiusofBigCircle+r*cosd((i-1)*separationAngle) 2*r+radiusofBigCircle+r*sind((i-1)*separationAngle)];
    points = [points;pointonCircle]; %Increment in the points array
end
end