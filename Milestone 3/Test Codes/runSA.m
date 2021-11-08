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
while(~SA.passInitialSolution)
    for i = 1:Robot.number
        Controller.controllers{1,i}.Waypoints = [Robot.initPosition(i,:)
            rand(Map.numberofPathsPoints-2,2) .* Map.size
            Map.goals(i,:)];
        SA.currentSol(:,:,i) = Controller.controllers{1,i}.Waypoints;
    end
    [pos,vel] = simulateKinematics(Robot,Controller,Map,false);
    SA.passInitialSolution = true;
    for i = 1:Robot.number
        SA.passInitialSolution = SA.passInitialSolution && checkPointFeasibility(squeeze(pos(:,:,1)),Map);
    end
    
    SA.cost = calculateCostFunction(pos,Robot);
    
end

for iteration = 1:SA.maxNumofIterations
    SA.passNewSolution = true;
    pos = [];
    vel = [];
    while(~SA.passNewSolution)
        for i = 1:Robot.number
            Controller.controllers{1,i}.Waypoints = [Robot.initPosition(i,:)
                (rand(Map.numberofPathsPoints-2,2) .* 2)-1
                Map.goals(i,:)];
        end
        for i = 1:Robot.number
            SA.passNewSolution = SA.passNewSolution && checkPointFeasibility(Controller.controllers{1,i}.Waypoints,Map);
        end
        if(SA.passNewSolution)
            return
        end
        [pos,vel] = simulateKinematics(Robot,Controller,Map,false);
        for i = 1:Robot.number
            SA.passInitialSolution = SA.passInitialSolution && checkPointFeasibility(squeeze(pos(:,:,1)),Map);
        end
    end
    SA.newCost = calculateCostFunction(pos,Robot);
    if(SA.newCost < SA.cost)
        SA.cost = SA.newCost;
        for i = 1:Robot.number
            SA.currentSol(:,:,i) = Controller.controllers{1,i}.Waypoints;
        end
    else
        Controller.controllers{1,i}.Waypoints = SA.currentSol(:,:,i);
    end
    disp(iteration,SA.cost)
end

% SA.initialSolution = [Robot.initiPositions
%                       rand(Map.numberofPathsPoints,2) .* Map.size
%