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
%% for normalization
distNorm = sum(sum(cellfun(@norm,num2cell(Robot.initPosition - 100*ones(Robot.number,2),2)) + cellfun(@norm,num2cell(Map.goals - 100*ones(Robot.number,2),2)) + norm(Map.size) * ones(Robot.number,2)));
[~,posSize,~] = size(pos);
[~,velSize,~] = size(vel);
cohesionNorm = Robot.number * posSize * (norm(Map.size)-Robot.rf);
velSet = [Controller.desiredLinearVelocity .* ones(1,ceil(Robot.number/2)) zeros(1,floor(Robot.number/2))];
stdNorm = std(velSet);
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
normalizedCohesionCost = cohesionCost/cohesionNorm;
%% Calculate the standard diviation for vx and vy
cell_x = num2cell(Matrix_vx',2);
cell_y = num2cell(Matrix_vy',2);
div_x = cellfun(@std,cell_x)./stdNorm;
div_y = cellfun(@std,cell_y)./stdNorm;
summdiv_x = sum(div_x)/velSize;
summdiv_y = sum(div_y)/velSize;
normalizedStdDeviation = 0.5*summdiv_x + 0.5*summdiv_y;
%% calculate the length of each robot path
for i = 1:Robot.number
    for j = 1:Map.numberofPathsPoints-1
        dist = dist + norm(Controller.controllers{1,i}.Waypoints(j,:)-Controller.controllers{1,i}.Waypoints(j+1,:)); 
    end
end
normalizedDist = dist/distNorm;
%% get the distance between each robot and the obstacle and get the summation of all of it
for i = 1:Robot.number
    for j = 1:N
        [distance,index] = closestObstacle(Map,Robot,pos(1:2,N,i));
        obsdist = (obsdist + ((distance - (Robot.originalRobotKinematics.TrackWidth + Map.radiusofObstacles(index)) - c)/(Map.farthestPointFromObstacle(index)-(Robot.originalRobotKinematics.TrackWidth + Map.radiusofObstacles(index)) - c)))/2;
    end
end
%% calculate the cost function by adding up all the objective functions  
% cost = 0 * -1 * obsdist + 100 * dist + 0 * summdiv_x + 0 * summdiv_y + cohesionCost;
cost = 1 * normalizedDist + 2 * normalizedCohesionCost + 1 * normalizedStdDeviation + -10 * obsdist;

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