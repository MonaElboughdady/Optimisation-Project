%Artificial Bee Colony (ABC) algorithm
%%Initialize ABC parameters
ABC.maxNumberOfIterations = 10; %Max number of iterations
ABC.colonySize = 20;  %the population size
ABC.limit = 3; %The max trials limit while exploiting on a solution
ABC.a = 10;
ABC.OnlookerBeesNumber = 10;
ABC.minScoutBees = 1;
%%Initialization Phase
%Starting by generating random paths for the robots
for j = 1:ABC.colonySize
    for i = 1:Robot.number
        ABC.population(j).controller{1,i} = controllerPurePursuit("DesiredLinearVelocity",...
            Controller.desiredLinearVelocity,"MaxAngularVelocity",Controller.maxAngularVelocity,...
            "LookaheadDistance",Controller.lookaheadDistance);
        ABC.population(j).controller{1,i}.Waypoints = min(max([Robot.initPosition(i,:)
            rand(Map.numberofPathsPoints-2,2) .* Map.size %generate random points
            Map.goals(i,:)],0),Map.size);
        ABC.suggestedPopulation(j).controller{1,i} = controllerPurePursuit("DesiredLinearVelocity",...
            Controller.desiredLinearVelocity,"MaxAngularVelocity",Controller.maxAngularVelocity,...
            "LookaheadDistance",Controller.lookaheadDistance);
        ABC.suggestedPopulation(j).controller{1,i}.Waypoints = min(max([Robot.initPosition(i,:)
            rand(Map.numberofPathsPoints-2,2) .* Map.size %generate random points
            Map.goals(i,:)],0),Map.size);
        ABC.OnlookerSuggestedSolution.controller{1,i} = controllerPurePursuit("DesiredLinearVelocity",...
            Controller.desiredLinearVelocity,"MaxAngularVelocity",Controller.maxAngularVelocity,...
            "LookaheadDistance",Controller.lookaheadDistance);
        ABC.OnlookerSuggestedSolution.controller{1,i}.Waypoints = min(max([Robot.initPosition(i,:)
            rand(Map.numberofPathsPoints-2,2) .* Map.size %generate random points
            Map.goals(i,:)],0),Map.size);
    end
    Controller.controllers = ABC.population(j).controller(1,:);
    [pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);
    ABC.fitness(j) = calculateCostFunction(pos,vel,Robot,Map,Controller,Robot.rf,Map.c);
end
ABC.abandonmentCounter = zeros(1,ABC.colonySize);
ABC.populationProbability = zeros(1,ABC.colonySize);
%%Iterate
for n = 1:ABC.maxNumberOfIterations
%Employed Bees Phase
    %Searching for New Food Sources
    for j = 1:ABC.colonySize
        currentWaypoints = cell2mat(cellfun(@(c) [c.Waypoints],ABC.population(j).controller,'UniformOutput',false));
        randomWaypoints = cell2mat(cellfun(@(c) [c.Waypoints],ABC.population(ceil(rand*ABC.colonySize)).controller,'UniformOutput',false));
        suggestedWaypoints = currentWaypoints + (-ABC.a + 2 * ABC.a *rand) * (currentWaypoints-randomWaypoints);
        for i = 1:Robot.number
            ABC.suggestedPopulation(j).controller{1,i}.Waypoints = suggestedWaypoints(:,2*i-1:2*i);
        end
        Controller.controllers = ABC.suggestedPopulation(j).controller(1,:);
        [pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);
        ABC.suggestedFitness(j) = calculateCostFunction(pos,vel,Robot,Map,Controller,Robot.rf,Map.c);
    end
    newBestIndices = (ABC.suggestedFitness - ABC.fitness) < 0;
    for j = 1:ABC.colonySize
        if newBestIndices(j) == 1
            ABC.population(j) = ABC.suggestedPopulation(j);
            ABC.fitness(j) = ABC.suggestedFitness(j);
            ABC.abandonmentCounter(j) = 0;
        else
            ABC.abandonmentCounter(j) = ABC.abandonmentCounter(j) + 1;
        end 
    end
%Onlooker Bees Phase
    [ABC.populationProbability, I] = ABC_calculateProbabilities(ABC.fitness,ABC.colonySize);
    for j = 1:ABC.OnlookerBeesNumber
        for i = 1:ABC.colonySize
            if(ABC.populationProbability(i)>rand)
                currentWaypoints = cell2mat(cellfun(@(c) [c.Waypoints],ABC.population(I(i)).controller,'UniformOutput',false));
                randomWaypoints = cell2mat(cellfun(@(c) [c.Waypoints],ABC.population(ceil(rand*ABC.colonySize)).controller,'UniformOutput',false));
                suggestedWaypoints = currentWaypoints + (-ABC.a + 2 * ABC.a *rand) * (currentWaypoints-randomWaypoints);
                for k = 1:Robot.number
                    ABC.OnlookerSuggestedSolution.controller{1,k}.Waypoints = suggestedWaypoints(:,2*k-1:2*k);
                end
                Controller.controllers = ABC.OnlookerSuggestedSolution.controller(1,:);
                [pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);
                ABC.OnlookerSuggestedFitness = calculateCostFunction(pos,vel,Robot,Map,Controller,Robot.rf,Map.c);
                if((ABC.OnlookerSuggestedFitness - ABC.fitness(I(i))) < 0)
                    ABC.population(I(i)) = ABC.OnlookerSuggestedSolution;
                    ABC.fitness(I(i)) = ABC.OnlookerSuggestedFitness;
                    ABC.abandonmentCounter(I(i)) = 0;
                    [ABC.populationProbability, I] = ABC_calculateProbabilities(ABC.fitness,ABC.colonySize);
                else
                    ABC.abandonmentCounter(I(i)) = ABC.abandonmentCounter(I(i)) + 1;
                end
                break
            end
        end    
    end
%Scout Bees Phase
ScoutBeesCount = 0;
    for j = 1:ABC.colonySize
        if(ABC.abandonmentCounter(j) >= ABC.limit)
            for i = 1:Robot.number
                ABC.population(j).controller{1,i}.Waypoints = min(max([Robot.initPosition(i,:)
                    rand(Map.numberofPathsPoints-2,2) .* Map.size %generate random points
                    Map.goals(i,:)],0),Map.size);
            end
            Controller.controllers = ABC.population(j).controller(1,:);
            [pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);
            ABC.fitness(j) = calculateCostFunction(pos,vel,Robot,Map,Controller,Robot.rf,Map.c);
            ABC.abandonmentCounter(j) = 0;
            ScoutBeesCount = ScoutBeesCount + 1;
        end
    end
    
    while(ScoutBeesCount<ABC.minScoutBees)
        [~,k] = max(ABC.abandonmentCounter);
        for i = 1:Robot.number
            ABC.population(k(1)).controller{1,i}.Waypoints = min(max([Robot.initPosition(i,:)
                rand(Map.numberofPathsPoints-2,2) .* Map.size %generate random points
                Map.goals(i,:)],0),Map.size);
        end
        Controller.controllers = ABC.population(k(1)).controller(1,:);
        [pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);
        ABC.fitness(k(1)) = calculateCostFunction(pos,vel,Robot,Map,Controller,Robot.rf,Map.c);
        ABC.abandonmentCounter(k(1)) = 0;
        ScoutBeesCount = ScoutBeesCount + 1;
    end
end
