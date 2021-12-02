%% Initialize the optimization problem parameters:
GA.Num_Generations = 50;  %maximum number of generations to be tested
GA.Pop_size = 20;  %the population size
GA.Elite_ratio = 0.1;  %percentage of survivors
GA.CrossOver_ratio = 0.5; %percentage of cross over processes
GA.Mutation_ratio = 1 - GA.Elite_ratio - GA.CrossOver_ratio; %the rest of mutation ratio
GA.Alpha = 0.7;  %alpha used for cross over process to generate new children
GA.Noise_Scale = 0.9;  %Used for mutation to add noise around specific genes, number from [0 to 1]
GA.Fitness = zeros(1,GA.Pop_size); %initaialize an empty fitness matrix
GA.Roulette = 0;% Roulette wheel selection on and off
GA.SUS = 1;  % Stochastic universial sampiling on and off
GA.TS = 0; %Tournament selection on and off
GA.K = 3; %the number of randomaly selected chromosoms in tournament selection
GA.Rank = 0; %Rank selection on and off
GA.Survivor = 1; % if 1 do fitness based selection, if 0 do Age based selection
GA.Age = zeros(1,GA.Pop_size); %initialize an array of zeros to store the ages
R = GA.CrossOver_ratio * GA.Pop_size; %define the number of desired selected parents
C = R; % define the number of childern
if rem(R,2) == 1 %check if R is odd number
    R = R+1;
end
Mut = GA.Mutation_ratio * GA.Pop_size; %define the number of the desires mutations
History = cell(GA.Num_Generations,1); %initialize an empty array of cells
Best_chromosomes = cell(GA.Num_Generations,1); %initialize an empty array of cells
Best_Fitness = zeros(1,GA.Num_Generations); %initialize an empty array to store the fitnesses
%% Initial step:
% Generate a random solution
GA.Population_int = cell(1,Robot.number);% initialize a matrix of initial population
GA.Population = cell(1,Robot.number); % initialize a matrix of population
for i = 1:GA.Pop_size %for all chromosoms
    for j = 1:Robot.number % for all genes
        GA.Population_int{j}(:,:,i) = min(max([Robot.initPosition(j,:)
            rand(Map.numberofPathsPoints-2,2).* Map.size
            Map.goals(j,:)],0),Map.size); % generate random chromosome
    end
end

%Calculate the fitness for first iteration
GA.Population = GA.Population_int; %initialize the population as equal to population initial
GA = fitness_calculation(GA,Robot,Map,Controller);
%% Iterations
for k = 1: GA.Num_Generations  % for number of generations
    disp("We are in generation :")
    disp(k)
    GA.Population_int = GA.Population; %update the previous population to the current population
    [GA.sortedFitness,indexes] = sort(GA.Fitness); %sort the fitness to get the index of the elite values
    GA.Elite_index = indexes(1: GA.Pop_size* GA.Elite_ratio); %get the indexes of the desired elite members
    GA.Mutation_index = indexes ( (1- GA.Mutation_ratio)* GA.Pop_size+1 : GA.Pop_size); % get the index of the the desired chromosomes to be mutated
    GA.Fitness_sum = sum(GA.Fitness); %summation of the fitness vector
    GA.parents_index = zeros(1,R); % an array of selected parents location on the population
    
    %Add the survivors
    for j = 1: GA.Pop_size* GA.Elite_ratio
        if (GA.Survivor == 1) %Elite based selection
            for i = 1: Robot.number
                GA.Population{i}(:,:,j) = GA.Population_int{i}(:,:,GA.Elite_index(j)); %add the elite members to the top of the new population
            end
        end
        if (GA.Survivor == 0) %age based selection
            random = randi([GA.Pop_size* GA.Elite_ratio+1, 50], [1, GA.Pop_size* GA.Elite_ratio]); %choose random childern to survive for the next the Generation
            for i = 1: Robot.number
                GA.Population{i}(:,:,j) = GA.Population_int{i}(:,:,random(j)); %add the survivor members to the top of the new population
            end
        end
    end
    
    %Generate the new children
    GA = Parents_selection(GA,R,indexes);%parents selection
    GA = Generate_offspring(GA,Robot,R,C); % Call Generating offspring
    GA = Mutation(GA,Mut,Robot,Map); %do mutation for the weak chromosomes
    
    %Calculate the fitness for each chromosome
    GA = fitness_calculation(GA,Robot,Map,Controller);
    
    %Data Storage
    [Best,indx] = sort(GA.Fitness); %get the best fitness of each population
    for i = 1:Robot.number
        Best_chromosomes{k}(:,:,i) = GA.Population{i}(:,:,indx(1)); %store the chromosomes with the best fitness value
        GA.best = GA.Population{i}(:,:,indx(1));
        Best_Fitness(1,k) = Best(1);
        GA.best_fitness = Best(1);
    end
    GA.bestFitnesses(k) = min(GA.Fitness);
    History{k} = GA; %story each GA with all its information in the History
    
    %plotPathsGA(Controller,Robot,Map,GA)
end