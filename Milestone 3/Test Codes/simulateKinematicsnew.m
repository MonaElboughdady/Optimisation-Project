function [posArray,velArray,failed] = simulateKinematicsnew(Robot,Controller,Map,visualize)
% tic;
failed = false;
sampleTime = Controller.sampleTime;
frameSize = Robot.originalRobotKinematics.TrackWidth/0.8;
vizRate = rateControl(10/sampleTime);
velArray = [];
posArray = [];
velTemp = [];
if(visualize)
    figure
    set(gcf, 'WindowState', 'maximized');
end

for i = 1:Robot.number
    robotCurrentPose(:,:,i) = Robot.initPose(:,i);
end

% for i = 1:Robot.number
%     velArray(:,:,i) = [0;0;0];
% end

cellfun(@reset,Controller.controllers);
for i = 1:Robot.number
    path = Controller.controllers{1,i}.Waypoints;
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
        
        
        if(visualize)
            hold off
            
            % Plot path each instance so that it stays persistent while robot mesh
            % moves
            plot(path(:,1), path(:,2),"k--d")
            hold all
            
            % Plot the path of the robot as a set of transforms
            plotTrVec = [robotCurrentPose(1:2); 0];
            plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
            plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
            light;
            xlim([0 55])
            ylim([0 55])
            
            waitfor(vizRate);
        end
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