function [paths] = generateNewSolution(oldPaths)

paths = oldPaths + 2.*rand(size(oldPaths))-1;

end