function GA_Visualization(GA,Robot,Controller,Map)
for j = 1:GA.Pop_size % for the size of the population 
        for i = 1:Robot.number 
            Controller.controllers{1,i}.Waypoints = GA.Population{i}(:,:,j); % assign the controller waypoints
        end
         visualize(Robot,Controller,Map);
end
end