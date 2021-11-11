% <<<<<<< Updated upstream
function [cost] = calculateCostFunction(pos,vel,Robot,Map,Controller,rf,c)

cohesionCost = 0;
Matrix_vx = vel(1,:,1);
Matrix_vy = vel(2,:,1);
for i = 1:Robot.number
    if(i>1)
        Matrix_vx = [Matrix_vx ;vel(1,:,i)];
        Matrix_vy = [Matrix_vy ;vel(2,:,i)];
    end
    if(i < Robot.number)
    for j = i+1:Robot.number
        if(i ~= j)
           posi_j = (pos(1:2,:,i)-pos(1:2, : ,j)).^2;
           ri_j = (posi_j(1,:) + posi_j(2,:));
           difference = abs(ri_j.^0.5 - rf);
           sumi_j = sum(difference);
           cohesionCost = cohesionCost + sumi_j;
        end
   end
    end 
end 
cell_x = num2cell(Matrix_vx',2);
cell_y = num2cell(Matrix_vy',2);

div_x = cellfun(@std,cell_x);
div_y = cellfun(@std,cell_y);
summdiv_x = sum(div_x);
summdiv_y = sum(div_y);

dist = 0;
for i = 1:Robot.number
    for j = 1:Map.numberofPathsPoints-1
        dist = dist + norm(Controller.controllers{1,i}.Waypoints(j,:)-Controller.controllers{1,i}.Waypoints(j+1,:)); 
    end
end
[M,N,O] = size(pos);
obsdist = 0;
for i = 1:Robot.number
    for j = 1:N
        [distance,index] = closestObstacle(Map,Robot,pos(1:2,N,i));
        obsdist = obsdist + (distance - (Robot.originalRobotKinematics.TrackWidth + Map.radiusofObstacles(index)) - c);
    end
end

cost = -1 * obsdist + dist + summdiv_x + summdiv_y + cohesionCost;
% =======
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