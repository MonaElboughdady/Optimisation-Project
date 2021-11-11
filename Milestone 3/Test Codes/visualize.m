function [posArray,velArray,failed] = visualize(Robot,Controller,Map)
% tic;
failed = false;
sampleTime = Controller.sampleTime;
frameSize = Robot.originalRobotKinematics.TrackWidth/0.8;
velArray = [];
posArray = [];
figure
set(gcf, 'WindowState', 'maximized');

for i = 1:Robot.number
    robotCurrentPose(:,:,i) = Robot.initPose(:,i);
end

% for i = 1:Robot.number
%     velArray(:,:,i) = [0;0;0];
% end

cellfun(@reset,Controller.controllers);
for i = 1:Robot.number
    currentTargetN = 1;
    currentTarget = Controller.controllers{1,i}.Waypoints(currentTargetN,:);
    velTemp = [];
    posTemp = [];
    robotCurrentPose = Robot.initPose(:,i);
    while(true)
        [v, omega] = Controller.controllers{1,i}(robotCurrentPose);
        vel = derivative(Robot.robots(i), robotCurrentPose, [v omega]);
        robotCurrentPose = robotCurrentPose + vel*sampleTime;
        distanceToNextTarget = norm(squeeze(robotCurrentPose(1:2))' - currentTarget);
        if(currentTarget == Controller.controllers{1,i}.Waypoints(end,:))
            if(distanceToNextTarget < Map.goalRadius)
                break
            end
        else
            if(distanceToNextTarget < Map.waypointRadius)
                currentTargetN = currentTargetN + 1;
                currentTarget = Controller.controllers{1,i}.Waypoints(currentTargetN,:);
            end
        end
        posTemp = [posTemp robotCurrentPose];
        velTemp = [velTemp vel];
        
        %         if(toc > 2)
        %             failed = true;
        %             return
        %         end
    end
    if(i > 1)
        [M,N] = size(posTemp);
        [P,Q] = size(posArray(:,:,i-1));
        if(N < Q)
            for j = 1:Q-N
                posTemp = [posTemp posTemp(:,end)];
            end
        elseif(N > Q)
            for j = 1:N-Q
                posArray(:,end+1,i-1) = posArray(:,end,i-1);
            end
        end
        [M,N] = size(velTemp);
        [P,Q] = size(velArray(:,:,i-1));
        if(N < Q)
            for j = 1:Q-N
                velTemp = [velTemp velTemp(:,end)];
            end
        elseif(N > Q)
            for j = 1:N-Q
                velArray(:,end+1,i-1) = velArray(:,end,i-1);
            end
        end
    end
    posArray(:,:,i) = posTemp;
    velArray(:,:,i) = velTemp;
end
[M,N,O] = size(posArray);
vizRate = rateControl(10/sampleTime);
for j = 1:N
        hold off
        
        % Plot path each instance so that it stays persistent while robot mesh
        % moves
        for i = 1:Robot.number
            if(i > 1)
                hold on
            end
            path = Controller.controllers{1,i}.Waypoints;
            plot(path(:,1), path(:,2),"k--d")
        end
        hold all
        
        % Plot the path of the robot as a set of transforms
        plotTrVec = [squeeze(posArray(1:2,j,:)); zeros(1,Robot.number)];
        plotRot = axang2quat([zeros(Robot.number,1) zeros(Robot.number,1) ones(Robot.number,1) squeeze(posArray(3,j,:))]);
        plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
        light;
        xlim([0 55])
        ylim([0 55])   
        waitfor(vizRate);
end
end