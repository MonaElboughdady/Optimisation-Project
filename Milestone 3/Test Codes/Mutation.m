function Mutation(GA,Mut,Robot)
%reset a random gene 
%[m,n] = size(GA.Mutation_index);
for i = 1:Mut
  random_Mut = randi([1,Robot.number]);  %choose a random gene to be mutated
  GA.Population{random_Mut}(:,:,n) = min(max([Robot.initPosition(random_Mut,:)
        rand(Map.numberofPathsPoints-2,2).* Map.size
        Map.goals(random_Mut,:)],0),Map.size); % generate random chromosom
end
end
