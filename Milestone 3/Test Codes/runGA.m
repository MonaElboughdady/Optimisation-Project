%% Initialize the optimization problem parameters:
GA.Num_Generations = 500;  %maximum number of generations to be tested
GA.Pop_size = 50;  %the population size
GA.Elite_ratio = 0.1;  %percentage of elitism
GA.CrossOver_ratio = 0.8; %percentage of cross over processes
GA.Mutation_ratio = 0.1; %the rest of mutation ratio
GA.Alpha = 0.4;  %alpha used for cross over process to generate new children
GA.Noise_Scale = 0.1;  %Used for mutation to add noise around specific genes, number from [0 to 1]
GA.Fitness = zeros(1,GA.Pop_size); %initaialize an empty fitness matrix
%% Initial step:
disp("Initial Solution")
% Generate a random solution
GA.Population_int = cell(Robot.number,1); % initialize a matrix of populations 
for i = 1:Robot.number
    for j = 1:GA.Pop_size % assign random genes for every person 
    GA.pop_int(:,:,j) = min(max([Robot.initPosition(i,:)
        rand(Map.numberofPathsPoints-2,2).* Map.size
        Map.goals(i,:)],0),Map.size); % matrix (numberofpaths x 2) add the persons into a matrix for poulation
    GA.pop_points_int(:,:,j) = GA.pop_int(:,:,j)';
    end
    GA.Population_int{i} = GA.pop_int; %assign an initial population for every robot
end

%Calculate the fitness of first generation
    for j = 1:GA.Pop_size % assign random genes for every person 
        for i = 1:Robot.number 
            Controller.controllers{1,i}.Waypoints = GA.Population_int{i}(:,:,j); %assign the waypoints
        end
    [pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);
    %GA.Population_int{1}(:,:,j).
    GA.Fitness(1,j) = calculateCostFunction(pos,vel,Robot,Map,Controller,Robot.rf,Map.c);
    end
    [sortedFitness,indexes] = sort(GA.Fitness); %sort the fitness to get the index of the elite values
    GA.Elite_index = indexes(1: GA.Pop_size* GA.Elite_ratio); %get the indexes of the desired elite members
    GA.Mutation_index = indexes ( (1- GA.Mutation_ratio)* GA.Pop_size : GA.Pop_size); % get the index of the the desired persons to be mutated

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    