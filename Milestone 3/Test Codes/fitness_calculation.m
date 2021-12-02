function GA = fitness_calculation(GA,Robot,Map,Controller)
    for j = 1:GA.Pop_size % for the size of the population 
        for i = 1:Robot.number 
            Controller.controllers{1,i}.Waypoints = GA.Population{i}(:,:,j); % assign the controller waypoints
        end
    [pos,vel] = simulateKinematicsnew(Robot,Controller,Map,false); %get the position and acceleration of each robot along the path
    GA.Fitness(1,j) = calculateCostFunction(pos,vel,Robot,Map,Controller,Robot.rf,Map.c); %get the cost value
    end
end
