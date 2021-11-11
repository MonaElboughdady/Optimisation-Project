function [posArray,velArray,failed] = simulateKinematicsnew(Robot,Controller,Map,visualize)
%this code is mainly used for calculating kinematics it's no longer used
%for visualiztion so the visualize flag is always set to false

failed = false;
sampleTime = Controller.sampleTime;
frameSize = Robot.originalRobotKinematics.TrackWidth/0.8; %frame size for visualization
vizRate = rateControl(10/sampleTime);
velArray = []; %intialize output and temp arrays
posArray = [];
velTemp = [];
if(visualize)
    figure
    set(gcf, 'WindowState', 'maximized');
end

for i = 1:Robot.number
    robotCurrentPose(:,:,i) = Robot.initPose(:,i); %setting current poses of robots
end

cellfun(@reset,Controller.controllers); %reset internal state of all controllers
for i = 1:Robot.number
    path = Controller.controllers{1,i}.Waypoints; %currnt path for visualization
    currentTargetN = 1; %index of currently tracked waypoint
    currentTarget = Controller.controllers{1,i}.Waypoints(currentTargetN,:); %currently tracked waypoint
    velTemp = [];%temp arrays
    posTemp = [];
    robotCurrentPose = Robot.initPose(:,i); %initial pose of current robot
    while(true)
        % Compute the controller outputs
        [v, omega] = Controller.controllers{1,i}(robotCurrentPose);
        % Get the robot's velocity using controller inputs
        vel = derivative(Robot.robots(i), robotCurrentPose, [v omega]);
        % Update the current pose
        robotCurrentPose = robotCurrentPose + vel*sampleTime;
        %calculate distance to next waypoint
        distanceToNextTarget = norm(squeeze(robotCurrentPose(1:2))' - currentTarget);
        if(currentTarget == Controller.controllers{1,i}.Waypoints(end,:)) %if the robot reaches the final waypoint i.e. target
            if(distanceToNextTarget < Map.goalRadius)
                break
            end
        else
            if(distanceToNextTarget < Map.waypointRadius) %if the robot reaches an intermediate waypoint
                currentTargetN = currentTargetN + 1;
                currentTarget = Controller.controllers{1,i}.Waypoints(currentTargetN,:);
            end
        end
        posTemp = [posTemp robotCurrentPose];
        velTemp = [velTemp vel];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %this part is no longer used here
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
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(i > 1)%this code pads the pos and vel arrays to have the same size as new or old arrays pos is concatenated with the last element and vel is padded with zeros
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