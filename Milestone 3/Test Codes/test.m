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