function [] = plotPaths(Controller,Robot,Map)
hold off
for i = 1:Robot.number
    if(i > 1)
        hold on
    end
    plot(Controller.controllers{1,i}.Waypoints(:,1),Controller.controllers{1,i}.Waypoints(:,2),"k--d")
end
for i = 1:Map.numberofObstacles
    plot(Map.locationofObstacles(1,i), Map.locationofObstacles(2,i), 'bo', 'MarkerSize', Map.radiusofObstacles(i))
end
xlim([0 100])
ylim([0 100])
drawnow
end