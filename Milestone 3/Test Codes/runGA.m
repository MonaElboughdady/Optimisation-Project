%% Initialize the optimization problem parameters:
GA.Num_Generations = 500;  %maximum number of generations to be tested
GA.Pop_size = 50;  %the population size
GA.Elite_ratio = 0.1;  %percentage of elitism
GA.CrossOver_ratio = 0.3; %percentage of cross over processes
GA.Mutation_ratio = 1 - GA.Elite_ratio - GA.CrossOver_ratio; %the rest of mutation ratio
GA.Alpha = 0.4;  %alpha used for cross over process to generate new children
GA.Noise_Scale = 0.1;  %Used for mutation to add noise around specific genes, number from [0 to 1]

%% Initialize a random population:
disp("Initial Solution")
GA.Population_int = zeros(Robot.number,1); % initialize a matrix of populations 
for i = 1:Robot.number
    for j = 1:GA.Pop_size
    Controller.controllers{1,j}.Waypoints = min(max([Robot.initPosition(i,:)
        rand(Map.numberofPathsPoints-2,2).* Map.size
        Map.goals(i,:)],0),Map.size);
        GA.pop_int(:,:,j) = Controller.controllers{1,j}.Waypoints;
    end
    GA.Population_int(i,1) = GA.pop_int(:,:,j);
end
[pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false);