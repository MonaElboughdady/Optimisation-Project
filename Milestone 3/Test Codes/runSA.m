SA.maxNumofIterations = 1000;
SA.initTemperature = 5000;
SA.finalTemperature = 1;
SA.currentTemperature = SA.initTemperature;
SA.temperatures = [];
SA.beta = (SA.initTemperature - SA.finalTemperature) / SA.maxNumofIterations;
SA.alpha = 0.8;
SA.linearCooling = true;
SA.checkProbability = 1;
SA.cost = 0;
SA.newCost = 0;
SA.costs = [];
SA.randomRange = 10;
figure
set(gcf, 'WindowState', 'maximized');
disp("Initial Solution")

for i = 1:Robot.number
    Controller.controllers{1,i}.Waypoints = min(max([Robot.initPosition(i,:)
        rand(Map.numberofPathsPoints-2,2) .* Map.size
        Map.goals(i,:)],0),Map.size);
    SA.currentSol(:,:,i) = Controller.controllers{1,i}.Waypoints;
    SA.bestSol(:,:,i) = Controller.controllers{1,i}.Waypoints;
end

[pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);
SA.cost = calculateCostFunction(pos,vel,Robot,Map,Controller,4,3);
SA.bestCost = SA.cost;


disp("Iterations")
for iteration = 1:SA.maxNumofIterations
    for i = 1:Robot.number
        Controller.controllers{1,i}.Waypoints = min(max(Controller.controllers{1,i}.Waypoints - [0 0
            (rand(Map.numberofPathsPoints-2,2) .* SA.randomRange) - (SA.randomRange/2)
            0 0],0),Map.size);
    end
    [pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);
    SA.newCost = calculateCostFunction(pos,vel,Robot,Map,Controller,4,3);
    SA.costDifference = SA.newCost - SA.cost;
    if(SA.costDifference < 0)
        SA.cost = SA.newCost;
        for i = 1:Robot.number
            SA.currentSol(:,:,i) = Controller.controllers{1,i}.Waypoints;
        end
    else
        SA.probability = exp(-SA.costDifference/SA.currentTemperature);
        if(SA.probability > rand)
            SA.cost = SA.newCost;
            for i = 1:Robot.number
                SA.currentSol(:,:,i) = Controller.controllers{1,i}.Waypoints;
            end
        else
            for i = 1:Robot.number
                Controller.controllers{1,i}.Waypoints = SA.currentSol(:,:,i);
            end
        end
    end
    if(SA.cost < SA.bestCost)
        SA.bestCost = SA.cost;
        for i = 1:Robot.number
            SA.bestSol(:,:,i) = Controller.controllers{1,i}.Waypoints;
        end
    end
    if(SA.linearCooling)
        SA.currentTemperature = SA.initTemperature - SA.beta * iteration;
    else
        SA.currentTemperature = SA.initTemperature * SA.beta ^ iteration;
    end
    SA.temperatures = [SA.temperatures SA.currentTemperature];
    plotPaths(Controller,Robot,Map,SA,iteration)
    SA.costs = [SA.costs SA.cost];
    disp(SA.cost)
    disp(iteration)
end