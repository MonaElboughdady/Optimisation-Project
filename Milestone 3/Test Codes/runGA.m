%% Initialize the optimization problem parameters:
GA.Num_Generations = 500;  %maximum number of generations to be tested
GA.Pop_size = 50;  %the population size
GA.Elite_ratio = 0.1;  %percentage of elitism
GA.CrossOver_ratio = 0.3; %percentage of cross over processes
GA.Mutation_ratio = 1 - GA.Elite_ratio - GA.CrossOver_ratio; %the rest of mutation ratio
GA.Alpha = 0.4;  %alpha used for cross over process to generate new children
GA.Noise_Scale = 0.1;  %Used for mutation to add noise around specific genes, number from [0 to 1]
GA.Fitness = zeros(1,GA.Pop_size); %initaialize an empty fitness matrix
GA.Roulette = 1;% Roulette wheel selection on and off
GA.SUS = 0;  % Stochastic universial sampiling on and off
GA.TS = 0; %Tournament selection on and off
GA.K = 10; %the number of randomaly selected chromosoms in tournament selection
GA.Rank = 0; %Rank selection on and off
GA.Survivor = 1; % if 1 do fitness based selection, if 0 do Age based selection
R = GA.CrossOver_ratio * GA.Pop_size; %define the number of desired selected parents
if rem(R,2) == 1 %check if R is odd number
R = R+1;
end
Mut = GA.Mutation_ratio * GA.Pop_size; %define the number of the desires mutations
%% Initial step:
% Generate a random solution
GA.Population_int = cell(Robot.number,1);% initialize a matrix of initial population 
GA.Population = cell(Robot.number,1); % initialize a matrix of population
for i = 1:GA.Pop_size %for all chromosoms
    for j = 1:Robot.number % for all genes 
    GA.Population_int{j}(:,:,i) = min(max([Robot.initPosition(j,:)
        rand(Map.numberofPathsPoints-2,2).* Map.size
        Map.goals(j,:)],0),Map.size); % generate random chromosome
    end
end

%Calculate the fitness for first iteration
GA.Population = GA.Population_int; %initialize the population as equal to population initial
fitness_calculation(GA,Robot,Map,Controller);
  
%% Iterations
for k = 1: GA.Num_Generations  % for number of generations
    GA.Population_int = GA.Population; %update the previous population to the current population
    [sortedFitness,indexes] = sort(GA.Fitness); %sort the fitness to get the index of the elite values
    GA.Elite_index = indexes(1: GA.Pop_size* GA.Elite_ratio); %get the indexes of the desired elite members
    GA.Mutation_index = indexes ( (1- GA.Mutation_ratio)* GA.Pop_size : GA.Pop_size); % get the index of the the desired chromosomes to be mutated
    GA.Fitness_sum = sum(GA.Fitness); %summation of the fitness vector
    GA.parents_index = zeros(1,R); % an array of selected parents location on the population
    Parents_selection(GA,R,indexes)%parents selection
    Generate_offspring(GA,Robot); % Call Generating off spring
    Mutation(GA,Mut,Robot) %do mutation for the weak chromosomes
    
    %Generat a new population
    if (GA.Survivor == 1)
    for i = 1: Robot.number
        for j = 1: GA.Pop_size
          
        end   
    end
    end
    
    %Calculate the fitness for each chromosome
    fitness_calculation(GA,Robot); 
end   
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    