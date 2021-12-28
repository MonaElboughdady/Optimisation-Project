function [] = plotPathsGA(Controller,Robot,Map,GA)
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
    viscircles([Map.locationofObstacles(1,i) Map.locationofObstacles(2,i)],Map.radiusofObstacles(i));
end
xlim([0 Map.size(1)])
ylim([0 Map.size(2)])
pbaspect([1 1 1])
title('Current Trajectory of Best Chromosome')
nexttile
plot(GA.bestFitnesses)
legend("Cost")
xlim([0 GA.Num_Generations])
ylim([-3 3])
pbaspect([1 1 1])
title('Cost')
nexttile
imagesc(GA.sortedFitness')
colorbar; colormap jet
pbaspect([1 1 1])
title('Population')
drawnow
end