function [feasible] = checkPointFeasibility(points,Map)

feasible = (all(points(1,:) < Map.size(:,1)) && all(points(2,:) < Map.size(:,2)) && all(points(2,:) > 0) && all(points(1,:) > 0));

end