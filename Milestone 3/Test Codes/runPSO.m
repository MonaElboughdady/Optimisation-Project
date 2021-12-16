PSO.w = 0.7;
PSO.c1 = 1.49;
PSO.c2 = 1.49;
PSO.r1 = 0.1;
PSO.r2 = 0.2;
PSO.fixedWeight = true;
PSO.topology = 1; %star topology 2: ring topolgy 3:four clusters 4:von neumann
PSO.populationSize = 50;
PSO.maxNumofIterations = 50;
PSO.fitness = zeros(1,PSO.populationSize);
PSO.initVelocities = cell(1,PSO.populationSize);
for i = 1:PSO.populationSize
    PSO.initVelocities{1,i} = zeros(Map.numberofPathsPoints,2,Robot.number);
end
PSO.previousVelocity = PSO.initVelocities;
%PSO.population = [];
%initialize pop
for j = 1:PSO.populationSize
    for i = 1:Robot.number
        PSO.population(j).controller{1,i} = controllerPurePursuit("DesiredLinearVelocity",...
            Controller.desiredLinearVelocity,"MaxAngularVelocity",Controller.maxAngularVelocity,...
            "LookaheadDistance",Controller.lookaheadDistance);
        PSO.population(j).controller{1,i}.Waypoints = min(max([Robot.initPosition(i,:)
            rand(Map.numberofPathsPoints-2,2) .* Map.size %generate random points
            Map.goals(i,:)],0),Map.size);
    end
end

for i = 1:PSO.populationSize
    Controller.controllers = PSO.population(i).controller(1,:);
    [pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);
    PSO.fitness(i) = calculateCostFunction(pos,vel,Robot,Map,Controller,Robot.rf,Map.c);
end
PSO.oldFitness = PSO.fitness;
PSO.personalBests = PSO.population;
[temp,I] = min(PSO.fitness);
PSO.bestSolutionFitness = temp;
PSO.bestSolution = PSO.population(I);
for n = 1:PSO.maxNumofIterations
    disp("Current Iteration:")
    disp(n)
    if(PSO.topology == 1)
        [~,I] = min(PSO.fitness);
        PSO.globalBest = PSO.population(I);
        for i =1:PSO.populationSize
            currentWaypoints = cell2mat(cellfun(@(c) [c.Waypoints],PSO.population(i).controller,'UniformOutput',false));
            personalBestWaypoints = cell2mat(cellfun(@(c) [c.Waypoints],PSO.personalBests(1).controller,'UniformOutput',false));
            globalBestWaypoints = cell2mat(cellfun(@(c) [c.Waypoints],PSO.globalBest.controller,'UniformOutput',false));
            vector1 = personalBestWaypoints - currentWaypoints;
            vector2 = globalBestWaypoints - currentWaypoints;
            r1 = rand;r2 = rand;
            PSO.currentVelocity{1,i} = PSO.w .* reshape(PSO.previousVelocity{1,i},Map.numberofPathsPoints,2*Robot.number) + PSO.c1 .* PSO.r1 .* vector1 + PSO.c2 .* PSO.r2 .* vector2;
            newWaypoints = currentWaypoints +  PSO.currentVelocity{1,i};
            for j = 1:Robot.number
                PSO.population(i).controller{1,j}.Waypoints = newWaypoints(:,2*j-1:2*j);
            end
        end
        
        for i = 1:PSO.populationSize
            Controller.controllers = PSO.population(i).controller(1,:);
            [pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);
            PSO.fitness(i) = calculateCostFunction(pos,vel,Robot,Map,Controller,Robot.rf,Map.c);
        end
        newPersonalBestIndices = (PSO.fitness - PSO.oldFitness) < 0;
        for i = 1:PSO.populationSize
            if newPersonalBestIndices(i) == 1
                PSO.personalBests(i) = PSO.population(i);
            end
        end
        PSO.oldFitness = PSO.fitness;
    end
    if(PSO.topology == 2)
        PSO.neighborhoodBest = [];
        for k = 1:PSO.populationSize
            if(k == 1)
                [~,I] = min([PSO.fitness(end) PSO.fitness(k) PSO.fitness(2)]);
                if I == 1; PSO.neighborhoodBest(k) = PSO.populationSize; elseif I == 3; PSO.neighborhoodBest(k) = 2; else; PSO.neighborhoodBest(k) = k; end
            elseif(k == PSO.populationSize)
                [~,I] = min([PSO.fitness(1) PSO.fitness(k) PSO.fitness(k-1)]);
                if I == 1; PSO.neighborhoodBest(k) = 1; elseif I == 3; PSO.neighborhoodBest(k) = k-1; else; PSO.neighborhoodBest(k) = k; end
            else
                [~,I] = min([PSO.fitness(k+1) PSO.fitness(k) PSO.fitness(k-1)]);
                if I == 1; PSO.neighborhoodBest(k) = k+1; elseif I == 3; PSO.neighborhoodBest(k) = k-1; else; PSO.neighborhoodBest(k) = k; end
            end
        end
        for i =1:PSO.populationSize
            currentWaypoints = cell2mat(cellfun(@(c) [c.Waypoints],PSO.population(i).controller,'UniformOutput',false));
            personalBestWaypoints = cell2mat(cellfun(@(c) [c.Waypoints],PSO.personalBests(1).controller,'UniformOutput',false));
            neighborhoodBestWaypoints = cell2mat(cellfun(@(c) [c.Waypoints],PSO.population(PSO.neighborhoodBest(i)).controller,'UniformOutput',false));
            vector1 = personalBestWaypoints - currentWaypoints;
            vector2 = neighborhoodBestWaypoints - currentWaypoints;
            r1 = rand;r2 = rand;
            PSO.currentVelocity{1,i} = PSO.w .* reshape(PSO.previousVelocity{1,i},Map.numberofPathsPoints,2*Robot.number) + PSO.c1 .* PSO.r1 .* vector1 + PSO.c2 .* PSO.r2 .* vector2;
            newWaypoints = currentWaypoints +  PSO.currentVelocity{1,i};
            for j = 1:Robot.number
                PSO.population(i).controller{1,j}.Waypoints = newWaypoints(:,2*j-1:2*j);
            end
        end
        
        for i = 1:PSO.populationSize
            Controller.controllers = PSO.population(i).controller(1,:);
            [pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);
            PSO.fitness(i) = calculateCostFunction(pos,vel,Robot,Map,Controller,Robot.rf,Map.c);
        end
        newPersonalBestIndices = (PSO.fitness - PSO.oldFitness) < 0;
        for i = 1:PSO.populationSize
            if newPersonalBestIndices(i) == 1
                PSO.personalBests(i) = PSO.population(i);
            end
        end
        PSO.oldFitness = PSO.fitness;
        
    end
    disp(PSO.fitness)
    %
    [best,I] = min(PSO.fitness);
    if(best < PSO.bestSolutionFitness)
        PSO.bestSolutionFitness = best;
        PSO.bestSolution = PSO.population(I);
    end
    Controller.controllers = PSO.population(I).controller(1,:);
    PSO.bestFitnesses(n) = best;
    Figures = plotPathsPSO(Figures,Robot,Map,pos,PSO);
end