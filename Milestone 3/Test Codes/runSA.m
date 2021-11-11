SA.maxNumofIterations = 1000;
SA.initTemperature = 5000;
SA.finalTemperature = 1;
SA.currentTemperature = SA.initTemperature;
SA.temperatures = [];
SA.beta = (SA.initTemperature - SA.finalTemperature) / SA.maxNumofIterations;
SA.alpha = 0.9;
SA.linearCooling = 1;
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
    SA.currentTemperature = SA.initTemperature - SA.beta * iteration;
    SA.temperatures = [SA.temperatures SA.currentTemperature];
    plotPaths(Controller,Robot,Map,SA,iteration)
    %         if(SA.newCost < SA.cost)
    %             SA.cost = SA.newCost;
    %             for i = 1:Robot.number
    %                 SA.currentSol(:,:,i) = Controller.controllers{1,i}.Waypoints;
    %             end
    %         else
    %             for i = 1:Robot.number
    %                 Controller.controllers{1,i}.Waypoints = SA.currentSol(:,:,i);
    %             end
    %         end
    SA.costs = [SA.costs SA.cost];
    %SA.randomRange = min(max(floor((exp(SA.cost)+0.5)*4),2),20);
    disp(SA.cost)
    disp(iteration)
end

%     SA.passInitialSolution = true;
%     for i = 1:Robot.number
%         SA.passInitialSolution = SA.passInitialSolution && checkPointFeasibility(squeeze(pos(:,:,1)),Map);
%     end

%
%     while(~SA.passNewSolution)
%         %         disp("Entered new solution")
%         SA.passNewSolution = true;
%         for i = 1:Robot.number
%             Controller.controllers{1,i}.Waypoints = min(max(Controller.controllers{1,i}.Waypoints - [0 0
%                 (rand(Map.numberofPathsPoints-2,2) .* 2)-1
%                 0 0],0),Map.size);
%         end
%         for i = 1:Robot.number
%             SA.passNewSolution = SA.passNewSolution && checkPathFeasibility(Controller.controllers{1,i}.Waypoints,Map);
%             SA.passNewSolution
%         end
%         if(~SA.passNewSolution)
%             continue
%         end
%         [pos,vel] = simulateKinematics(Robot,Controller,Map,false);
%         for i = 1:Robot.number
%             SA.passInitialSolution = SA.passInitialSolution && checkPointFeasibility(squeeze(pos(:,:,1)),Map);
%         end
%     end

% SA.initialSolution = [Robot.initiPositions
%                       rand(Map.numberofPathsPoints,2) .* Map.size
%