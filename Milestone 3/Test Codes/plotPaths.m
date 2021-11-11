function [] = plotPaths(Controller,Robot,Map,SA,iteration)
tiledlayout(1,3)
nexttile
hold off
for i = 1:Robot.number
    if(i > 1)
        hold on
    end
    plot(Controller.controllers{1,i}.Waypoints(:,1),Controller.controllers{1,i}.Waypoints(:,2),"k--d",'color',Robot.pathsColors(i,:),'DisplayName',['Robot' num2str(i)])
end
legend('AutoUpdate','off')
for i = 1:Map.numberofObstacles
    plot(Map.locationofObstacles(1,i), Map.locationofObstacles(2,i), 'bo', 'MarkerSize', Map.radiusofObstacles(i),'DisplayName','')
end
xlim([0 Map.size(1)])
ylim([0 Map.size(2)])
pbaspect([1 1 1])
nexttile
plot(SA.costs)
legend("Cost")
xlim([0 SA.maxNumofIterations])
ylim([-1e6 1e5])
pbaspect([1 1 1])
nexttile
plot(SA.temperatures)
legend("Temperature")
xlim([0 SA.maxNumofIterations])
ylim([0 SA.initTemperature])
pbaspect([1 1 1])
drawnow
end