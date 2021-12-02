function GA = Parents_selection(GA,R,indexes)
%% Initialization for Fitness propotionate selection
        probability = zeros(1,GA.Pop_size); % define an array for the probability of each chromosom
        Q = zeros(1,GA.Pop_size);
        Fitness = 1./GA.Fitness;
        Fitness_sum = sum(Fitness);
        for count = 1 : GA.Pop_size %for the size of population
        probability(1,count) = (Fitness(count)/Fitness_sum); %P(count) = the probability of solution count'th
        Q(count) = sum(probability); %Q(count) = cumulative probability of the count'th solution
        end
   
%% Roulette selection
    if (GA.Roulette == 1)
        for i = 1 : R
            random = rand(1); %generate a random number each iteration till R times
            %If Rand is less than Q1, the first solution (X1) is selected; otherwise the jth solution 
            %is selected such that Rand is greater than Qj‐1 and less or equal than Qj (Qj 1 Rand Qj).
            if (random < Q(1))
               GA.parents_index(i) = 1; %choose the parent index and add to an array
            end
            for j = 2 : GA.Pop_size
                if((Q(j-1)<random) && (Q(j)>=random))
                   GA.parents_index(i) = j; 
                end
            end
        end
    end
%% SUS selection
    if(GA.SUS == 1)
        %If Rand is less than Q1, the first solution (X1) is selected; otherwise the jth solution 
        %is selected such that Rand is greater than Qj‐1 and less or equal than Qj (Qj 1 Rand Qj).
        SUS_IND = 1/R;
        Pointer = SUS_IND * rand(1); %generate a random number between 0 and the SUS index 
         for i = 1 : R
            if (Pointer < Q(1))
               GA.parents_index(i) = 1; 
            end
            for j = 2 : GA.Pop_size
                if((Q(j-1)<Pointer) && (Q(j)>=Pointer))
                   GA.parents_index(i) = j; 
                end
            end
            Pointer = Pointer + SUS_IND;
         end
    end
    
%% Tournament selection
     if(GA.TS == 1)
        GA.random_array = zeros(1,GA.K);  
        for j = 1:R %for number of required parents
            GA.random_indexes = randi([1,GA.Pop_size],[1,GA.K]); % get k random chromosoms to compare in a tournament
            for i = 1:GA.K % for the K as is the number of randomaly selected chromosoms
           GA.random_array(i) = GA.Fitness(GA.random_indexes(i)); 
            end
        [M,I] = min(GA.random_array); % get the index of the minimum fitness value and store in I
         I = GA.random_indexes(I);
        GA.parents_index(j) = I; %store the index of the best parent 
        end 
    end

%% Rank selection  
    if(GA.Rank == 1)
       GA.parents_index = indexes(1:R); %choose the members with highest rank as parents
    end

end