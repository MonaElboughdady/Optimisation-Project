function [posArray,velArray] = simulateKinematics(Robot,Controller,Map,visualize)
tic;
disp("Entered Simulation")
goalRadius = Map.goalRadius;
sampleTime = Controller.sampleTime;
currentWaypoints = 2*ones(1,Robot.number);
currentWaypoint = zeros(Robot.number,2);
finalWaypoint = zeros(Robot.number,1);
initialize = true;
for i = 1:Robot.number
    currentWaypoint(i,:) = Controller.controllers{1,i}.Waypoints(2,:);
end
distanceToGoal = cellfun(@norm,num2cell(Robot.initPosition - currentWaypoint,2));
frameSize = Robot.originalRobotKinematics.TrackWidth/0.8;
vizRate = rateControl(10/sampleTime);
velArray = [];
posArray = [];
if(visualize)
    figure
    set(gcf, 'WindowState', 'maximized');
end

for i = 1:Robot.number
    robotCurrentPose(:,:,i) = Robot.initPose(:,i);
end
for i = 1:Robot.number
    [v, omega] = Controller.controllers{1,i}(robotCurrentPose(:,:,i));
    vel(:,:,i) = derivative(Robot.robots(i), robotCurrentPose(:,:,i), [v omega]);
end

while(any(distanceToGoal > goalRadius) || ~all(finalWaypoint))
   
    for i = 1:Robot.number
        
        Controller.controllers{1,i}.Waypoints = Controller.controllers{1,i}.Waypoints;
        if(currentWaypoints(i) < Map.numberofPathsPoints)
            goalRadius = Map.waypointRadius;
        else
            goalRadius = Map.goalRadius;
        end
        %         if(i == 2)
        %             distanceToGoal(i) < goalRadius
        %             ~finalWaypoint(i)
        %         end
        %         finalWaypoint
        currentWaypoints;
        if(distanceToGoal(i) < goalRadius && ~finalWaypoint(i))
            %             finalWaypoint
            %             currentWaypoints
            currentWaypoints(i) = currentWaypoints(i) + 1;
            if(currentWaypoints(i) > Map.numberofPathsPoints)
                finalWaypoint(i) = true;
                continue
            end
            currentWaypoint(i,:) = Controller.controllers{1,i}.Waypoints(currentWaypoints(i),:);
            distanceToGoal(i) = norm(squeeze(robotCurrentPose(1:2,:,i))' - currentWaypoint(i,:));
        elseif(~finalWaypoint(i))
            %             if(i~=3)
            %                 distanceToGoal(3)
            %             end
            Controller.controllers{1,i}.Waypoints;
            [v, omega] = Controller.controllers{1,i}(robotCurrentPose(:,:,i));
            vel(:,:,i) = derivative(Robot.robots(i), robotCurrentPose(:,:,i), [v omega]);
            %         velArray(:,:,i) = vel;
            robotCurrentPose(:,:,i) = robotCurrentPose(:,:,i) + vel(:,:,i)*sampleTime;
            %         posArray(:,:,i) = robotCurrentPose;
            distanceToGoal(i) = norm(squeeze(robotCurrentPose(1:2,:,i))' - currentWaypoint(i,:));
        end
    end
    
    velArray = [velArray vel];
    posArray = [posArray robotCurrentPose];
    
    if(visualize)
        
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
        plotTrVec = [robotCurrentPose(1:2,:); zeros(1,Robot.number)];
        plotRot = axang2quat([zeros(Robot.number,1) zeros(Robot.number,1) ones(Robot.number,1) squeeze(robotCurrentPose(3,:,:))]);
        plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
        light;
        xlim([0 max(Map.goals(:,1)) + 40])
        ylim([0 max(Map.goals(:,1)) + 40])
        waitfor(vizRate);
    end
    goalRadius = Map.goalRadius;
    if(toc > 3)
        return
    end
end
end