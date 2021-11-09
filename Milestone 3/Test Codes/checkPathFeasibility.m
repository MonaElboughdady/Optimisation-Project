function [feasible] = checkPathFeasibility(paths,Map)

feasible = (all(paths(:,1) < Map.size(:,1)) && all(paths(:,2) < Map.size(:,2)) && all(paths(:,1) > 0) && all(paths(:,2) > 0));

end