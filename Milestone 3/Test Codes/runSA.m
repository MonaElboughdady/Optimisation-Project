%Simulated Annealing Algorithm
%Initialize SA Parameters
SA.maxNumofIterations = 100; %Max number of iterations
SA.initTemperature = 5000; %Initial Temperature
SA.finalTemperature = 1; %Final Temperature
SA.currentTemperature = SA.initTemperature; %Current Temperature 
SA.temperatures = []; %Temperatures Array
SA.beta = (SA.initTemperature - SA.finalTemperature) / SA.maxNumofIterations; %Linear Cooling Coefficient Beta
SA.alpha = 0.8; %Geometric Cooling Coefficient alpha
SA.linearCooling = false; %Linear Cooling Flag
SA.checkProbability = 1; %Check probability Flag
SA.cost = 0; %Simulated Annealing Cost
SA.newCost = 0; %Simulated Annealing New Cost
SA.costs = []; %Simulated Annealing Costs Array
SA.randomRange = 10; %Random Range
figure
set(gcf, 'WindowState', 'maximized');
disp("Initial Solution")

%Generate initial solution for each robot
for i = 1:Robot.number
    Controller.controllers{1,i}.Waypoints = min(max([Robot.initPosition(i,:)
        rand(Map.numberofPathsPoints-2,2) .* Map.size %????
        Map.goals(i,:)],0),Map.size);
    SA.currentSol(:,:,i) = Controller.controllers{1,i}.Waypoints; %Setting the current solution
    SA.bestSol(:,:,i) = Controller.controllers{1,i}.Waypoints;  %Setting the best solution till now
end

[pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);
SA.cost = calculateCostFunction(pos,vel,Robot,Map,Controller,Robot.rf,Map.c); %Calculating the initial cost function
SA.bestCost = SA.cost;  %Setting the best cost till now

disp("Iterations")

for iteration = 1:SA.maxNumofIterations %Looping till the max number of iterations reached
    for i = 1:Robot.number %Generate a random solution for each robot
        Controller.controllers{1,i}.Waypoints = min(max(Controller.controllers{1,i}.Waypoints - [0 0
            (rand(Map.numberofPathsPoints-2,2) .* SA.randomRange) - (SA.randomRange/2)
            0 0],0),Map.size);
    end
    [pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);
    SA.newCost = calculateCostFunction(pos,vel,Robot,Map,Controller,Robot.rf,Map.c); %Calculating the current cost function
    SA.costDifference = SA.newCost - SA.cost; %Calculating the difference between the current cost and the previous one
    if(SA.costDifference < 0) %Check if new solution is better than old one
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