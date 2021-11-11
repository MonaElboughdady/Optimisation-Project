function [] = plotPaths(Controller,Robot)
hold on
for i = 1:Robot.number
    plot(Controller.controllers{1,i}.Waypoints(:,1),Controller.controllers{1,i}.Waypoints(:,2),"k--d")
end
hold off
end