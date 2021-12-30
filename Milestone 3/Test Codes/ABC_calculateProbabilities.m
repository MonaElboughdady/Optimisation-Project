function [Q, I] = ABC_calculateProbabilities(fitness,colonySize)
[sortedFitness, I] = sort(fitness,'descend');
NewFitness = 1./sortedFitness;
fitnessSum = sum(NewFitness);
probability = zeros(1,colonySize); % define an array for the probability of each chromosom
Q = zeros(1,colonySize);
for count = 1 : colonySize %for the size of population
    probability(1,count) = (NewFitness(count)/fitnessSum); %P(count) = the probability of solution count'th
    Q(count) = sum(probability); %Q(count) = cumulative probability of the count'th solution
end

end