function [Figures] = plotPathsABC(Figures,Robot,Map,pos,ABC)
figure(Figures.runningFigure);
persistent numberOfExecutions
if isempty(numberOfExecutions)
    numberOfExecutions = 0;
    subplot(1,2,1);
    hold off;
    for i = 1:Robot.number
        if(i > 1)
            hold on
        end
        Figures.trajectory(1,i) = plot(pos(1,:,i),pos(2,:,i),'color',Robot.pathsColors(i,:),'DisplayName',['Robot' num2str(i)]);
    end
    legend('AutoUpdate','off')
    for i = 1:Map.numberofObstacles
        plot(Map.locationofObstacles(1,i), Map.locationofObstacles(2,i), 'bo', 'MarkerSize', Map.radiusofObstacles(i),'DisplayName','')
    end
    xlim([0 Map.size(1)])
    ylim([0 Map.size(2)])
    axis square;
    title('Current Trajectory of Best Particle')
    subplot(1,2,2);
    Figures.fitness = plot(ABC.bestFitnesses);
    legend("Cost")
    title('Cost of best particle')
    xlim([0 ABC.maxNumberOfIterations])
    ylim([0 4])
    drawnow
end
subplot(1,2,1);
for i = 1:Robot.number
    set(Figures.trajectory(1,i),'XData',pos(1,:,i),'YData',pos(2,:,i))
end
set(Figures.fitness,'YData',ABC.bestFitnesses)
drawnow
end