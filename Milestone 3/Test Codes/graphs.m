function [obsdist] = graphs(Robot,Map,pos,vel,rf,c)
tiledlayout(1,3)
nexttile
pbaspect([1 1 1])
title('r_{ij} - r_f. The Difference between parameter' ,'r_f and the distance between each 2 robots (less is better).')
if(Robot.number ~=0 )
    Matrix_vx = vel(1,:,1); %initialize a matrix that will contain all the values of velocities
    % in the x direction
    Matrix_vy = vel(2,:,1); %initialize a matrix that will contain all the values of velocities
    % in the y direction
end
for i = 1:Robot.number %Loop over each robot
    if(i>1)
        Matrix_vx = [Matrix_vx ;vel(1,:,i)]; %Get the longitudinal velocity matrix
        Matrix_vy = [Matrix_vy ;vel(2,:,i)]; %Get the lateral velocity matrix
    end
    hold on
    if(i < Robot.number)
        for j = i+1:Robot.number
            if(i ~= j)
                posi_j = (pos(1:2,:,i)-pos(1:2, : ,j)).^2; %get the difference between the position robot i and all next robots j in x and y then square it
                ri_j = (posi_j(1,:) + posi_j(2,:)); %add the squared x and y position
                difference = abs(ri_j.^0.5 - rf);
                plot(difference,'DisplayName',['Robot' num2str(i) '->' num2str(j)])
            end
        end
    end
end
legend
nexttile
pbaspect([1 1 1])
title('Standard deviation of velocity during simulation' ,'(less is better).')
cell_x = num2cell(Matrix_vx',2);
cell_y = num2cell(Matrix_vy',2);
div_x = cellfun(@std,cell_x);
div_y = cellfun(@std,cell_y);
hold on
plot(div_x,'DisplayName','V_x')
plot(div_y,'DisplayName','V_y')
legend
nexttile
pbaspect([1 1 1])
title('Safe distance between robot and its closest obstacle' ,'(more is better).')
[M,N,O] = size(pos);
obsdist = zeros(O,N);
for i = 1:Robot.number
    for j = 1:N
        [distance,index] = closestObstacle(Map,Robot,pos(1:2,j,i));
        obsdist(i,j) = (distance - (Robot.originalRobotKinematics.TrackWidth + Map.radiusofObstacles(index)) - c);
    end
end
hold on
for i = 1:Robot.number
    plot(obsdist(i,:),'DisplayName',['Robot' num2str(i)])
end
legend
end