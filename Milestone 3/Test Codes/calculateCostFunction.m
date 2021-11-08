function [cost] = calculateCostFunction(pos,vel,Robot,rf)

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
cell_x = num2cell(Matrix_x',2);
cell_y = num2cell(Matrix_y',2);

div_x = cellfun(@std,cell_x);
div_y = cellfun(@std,cell_y);
summdiv_x = sum(div_x);
summdiv_y = sum(div_y);

cost = summdiv_x + summdiv_y + cohesionCost;
end