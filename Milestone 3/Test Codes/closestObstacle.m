function [distance,index] = closestObstacle(Map,Robot,position)
distance = [];
for i = 1:Robot.number
    distance = cellfun(@norm,num2cell((position - Map.locationofObstacles)',2));
end
[distance,index] = min(distance);
end
