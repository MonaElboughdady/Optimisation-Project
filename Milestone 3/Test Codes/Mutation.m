function GA = Mutation(GA,Mut,Robot,Map)
%reset a random gene 
%[m,n] = size(GA.Mutation_index);
GA.Mutation_elements = cell(1,Robot.number);
for i = 1:Robot.number
    for j = 1:Mut
      GA.Mutation_elements{i}(:,:,j) =  GA.Population_int{i}(:,:,GA.Mutation_index(j)); %put the weak members in a mutation 3d array 
    end
end
for i = 1:Mut
  random_Mut = randi([1,Robot.number]);  %choose a random gene to be mutated
  GA.Mutation_elements{random_Mut}(:,:,i) = min(max([Robot.initPosition(random_Mut,:)
        rand(Map.numberofPathsPoints-2,2).* Map.size
        Map.goals(random_Mut,:)],0),Map.size); % generate random chromosome
end
for i = 1:Robot.number
   GA.Population{i}(:,:,(1- GA.Mutation_ratio)* GA.Pop_size+1:GA.Pop_size) = GA.Mutation_elements{i}(:,:,:); %add the mutation to the new generation
end
end
