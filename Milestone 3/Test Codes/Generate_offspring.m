function Generate_offspring(GA,Robot,R)
GA.Offspring = cell(Robot.number,1);
for i = 1:2:R %make R/2 crossovers
    for j = 1:Robot.number
      GA.Offspring{j}(:,:,i) = GA.Alpha * GA.Population_int{j}(:,:,GA.parents_index(i)) + (1-GA.Alpha) * GA.Population_int{j}(:,:,GA.parents_index(i+1)); %first child of the crossover
      GA.Offspring{j}(:,:,i+1) = GA.Alpha * GA.Population_int{j}(:,:,GA.parents_index(i+1)) + (1-GA.Alpha) * GA.Population_int{j}(:,:,GA.parents_index(i)); %second child of the crossover
    end
end
end