function GA = Generate_offspring(GA,Robot,R,C)
GA.Offspring = cell(1,Robot.number);
for i = 1:2:R %make R/2 crossovers
    for j = 1:Robot.number
      GA.Offspring{j}(:,:,i) = GA.Alpha * GA.Population_int{j}(:,:,GA.parents_index(i)) + (1-GA.Alpha) * GA.Population_int{j}(:,:,GA.parents_index(i+1)); %first child of the crossover
      GA.Offspring{j}(:,:,i+1) = GA.Alpha * GA.Population_int{j}(:,:,GA.parents_index(i+1)) + (1-GA.Alpha) * GA.Population_int{j}(:,:,GA.parents_index(i)); %second child of the crossover
    end
end
for i = 1:Robot.number
   GA.Population{i}(:,:,(GA.Pop_size* GA.Elite_ratio)+1:(GA.Pop_size* GA.Elite_ratio)+C) = GA.Offspring{i}(:,:,1:C); %add the children to the new generation
end
end