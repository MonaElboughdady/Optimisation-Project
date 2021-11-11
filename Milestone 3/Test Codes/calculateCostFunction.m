% <<<<<<< Updated upstream
function [cost] = calculateCostFunction(pos,vel,Robot,Map,Controller,rf,c)
%% initialize variables
cohesionCost = 0;
if(Robot.number ~=0 )
 Matrix_vx = vel(1,:,1); %initialize a matrix that will contain all the values of velocities
 % in the x direction
 Matrix_vy = vel(2,:,1); %initialize a matrix that will contain all the values of velocities
 % in the y direction   
end
dist = 0;
obsdist = 0;
[M,N,O] = size(pos);
%% calculate the distance between each robot and all the successive robots
for i = 1:Robot.number %Loop over each robot
    if(i>1) 
        Matrix_vx = [Matrix_vx ;vel(1,:,i)]; %Get the longitudinal velocity matrix
        Matrix_vy = [Matrix_vy ;vel(2,:,i)]; %Get the lateral velocity matrix
    end
    if(i < Robot.number)
    for j = i+1:Robot.number
        if(i ~= j)
           posi_j = (pos(1:2,:,i)-pos(1:2,:,j)).^2; %get the difference between the position robot i and all next robots j in x and y then square it
           ri_j = (posi_j(1,:) + posi_j(2,:)); %add the squared x and y position
           difference = abs(ri_j.^0.5 - rf); 
           sumi_j = sum(difference); %sum differences
           cohesionCost = cohesionCost + sumi_j; % add all the differences to get the objective function for the distances between the robots
        end
     end
   end 
end 
%% Calculate the standard diviation for vx and vy
cell_x = num2cell(Matrix_vx',2);
cell_y = num2cell(Matrix_vy',2);
div_x = cellfun(@std,cell_x);
div_y = cellfun(@std,cell_y);
summdiv_x = sum(div_x);
summdiv_y = sum(div_y);

%% calculate the length of each robot path
for i = 1:Robot.number
    for j = 1:Map.numberofPathsPoints-1
        dist = dist + norm(Controller.controllers{1,i}.Waypoints(j,:)-Controller.controllers{1,i}.Waypoints(j+1,:)); 
    end
end

%% get the distance between each robot and the obstacle and get the summation of all of it
for i = 1:Robot.number
    for j = 1:N
        [distance,index] = closestObstacle(Map,Robot,pos(1:2,N,i));
        obsdist = obsdist + (distance - (Robot.originalRobotKinematics.TrackWidth + Map.radiusofObstacles(index)) - c);
    end
end
%% calculate the cost function by adding up all the objective functions  
cost = -1 * obsdist + dist + summdiv_x + summdiv_y + cohesionCost;

%% Draft
% function [cost] = calculateCostFunction(pos,Robot)
% 
% cohesionCost = 0;
% for i = 1:Robot.number
%     for j = 1:Robot.number
%         if(i ~= j)
%             cohesionCost = cohesionCost + sum(abs(cellfun(@norm,num2cell((pos(1:2,:,i) - pos(1:2,:,j))',2))-5));
%         end
%     end    
% end
% 
% cost = cohesionCost;
% 
% >>>>>>> Stashed changes
% end