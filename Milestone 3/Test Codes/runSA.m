SA.maxNumofIterations = 1000;
SA.initTemperature = 1000;
SA.finalTemperature = 1;
SA.beta = (SA.initTemperature - SA.finalTemperature) / SA.maxNumofIterations;
SA.alpha = 0.9;
SA.linearCooling = 1;
SA.checkProbability = 1;
SA.passInitialSolution = false;
SA.passNewSolution = false;
SA.cost = 0;
SA.newCost = 0;
SA.costs = [];
SA.randomRange = 20;
figure
set(gcf, 'WindowState', 'maximized');
disp("Initial Solution")
while(~SA.passInitialSolution)
    for i = 1:Robot.number
        Controller.controllers{1,i}.Waypoints = min(max([Robot.initPosition(i,:)
            rand(Map.numberofPathsPoints-2,2) .* Map.size
            Map.goals(i,:)],0),Map.size);
        SA.currentSol(:,:,i) = Controller.controllers{1,i}.Waypoints;
    end
    [pos,vel] = simulateKinematics(Robot,Controller,Map,false);
%     SA.passInitialSolution = true;
%     for i = 1:Robot.number
%         SA.passInitialSolution = SA.passInitialSolution && checkPointFeasibility(squeeze(pos(:,:,1)),Map);
%     end
    
    SA.cost = calculateCostFunction(pos,vel,Robot,Map,Controller,4);
    
end

disp("Iterations")
for iteration = 1:SA.maxNumofIterations
    
    for i = 1:Robot.number
        Controller.controllers{1,i}.Waypoints = min(max(Controller.controllers{1,i}.Waypoints - [0 0
            (rand(Map.numberofPathsPoints-2,2) .* SA.randomRange)-floor(SA.randomRange/2)
            0 0],0),Map.size);
    end
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
    [pos,vel] = simulateKinematics(Robot,Controller,Map,false);
    SA.newCost = calculateCostFunction(pos,vel,Robot,Map,Controller,4);
    if(SA.newCost < SA.cost)
        SA.cost = SA.newCost;
        for i = 1:Robot.number
            SA.currentSol(:,:,i) = Controller.controllers{1,i}.Waypoints;
        end
    else
        Controller.controllers{1,i}.Waypoints = SA.currentSol(:,:,i);
    end
    SA.costs = [SA.costs SA.cost];
    SA.randomRange = min(max(floor(log10(SA.cost)*4),2),20);
    disp(iteration)
end

% SA.initialSolution = [Robot.initiPositions
%                       rand(Map.numberofPathsPoints,2) .* Map.size
%