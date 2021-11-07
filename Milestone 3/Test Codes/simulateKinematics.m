function [posArray,velArray] = simulateKinematics(Robot,Controller,Map,visualize)
goalRadius = Map.goalRadius;
sampleTime = Controller.sampleTime;
distanceToGoal = cellfun(@norm,num2cell(Robot.initPosition - Map.goals,2));
frameSize = Robot.originalRobotKinematics.TrackWidth/0.8;
% path = Controller.controllers{1,1}.Waypoints;
flag = true;
vizRate = rateControl(10/sampleTime);
figure
set(gcf, 'WindowState', 'maximized');
for i = 1:Robot.number
    robotCurrentPose(:,:,i) = Robot.initPose(:,i);
end
while(distanceToGoal > goalRadius)
    
    for i = 1:Robot.number
        if(distanceToGoal(i) > goalRadius)
            Controller.controllers{1,i}.Waypoints;
            [v, omega] = Controller.controllers{1,i}(robotCurrentPose(:,:,i));
            vel(:,:,i) = derivative(Robot.robots(i), robotCurrentPose(:,:,i), [v omega]);
            %         velArray(:,:,i) = vel;
            robotCurrentPose(:,:,i) = robotCurrentPose(:,:,i) + vel(:,:,i)*sampleTime;
            %         posArray(:,:,i) = robotCurrentPose;
            distanceToGoal = cellfun(@norm,num2cell(squeeze(robotCurrentPose(1:2,:,i))' - Map.goals,2));
        else
            distanceToGoal(i) > goalRadius
            i
            continue
        end
        
    end
    if(flag)
        velArray = vel;
        posArray = robotCurrentPose;
        flag = false;
    else
        velArray = [velArray vel];
        posArray = [posArray robotCurrentPose];
    end
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
%     plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2,:); zeros(1,Robot.number)];
    plotRot = axang2quat([zeros(Robot.number,1) zeros(Robot.number,1) ones(Robot.number,1) squeeze(robotCurrentPose(3,:,:))]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 40])
    ylim([0 40])
    
    waitfor(vizRate);
    
    
    
    %     % Compute the controller outputs, i.e., the inputs to the robot
    %     [v, omega] = controller(robotCurrentPose);
    %
    %     % Get the robot's velocity using controller inputs
    %     vel = derivative(robot, robotCurrentPose, [v omega]);
    %     velarray = [velarray vel];
    %     % Update the current pose
    %     robotCurrentPose = robotCurrentPose + vel*sampleTime;
    %     poseArray = [poseArray robotCurrentPose];
    %     % Re-compute the distance to the goal
    %     distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    %
    %     hold off
    %
    %     % Plot path each instance so that it stays persistent while robot mesh
    %     % moves
    %     plot(path(:,1), path(:,2),"k--d")
    %     hold all
    %
    %     % Plot the path of the robot as a set of transforms
    %     plotTrVec = [robotCurrentPose(1:2); 0];
    %     plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    %     plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    %     light;
    %     xlim([0 35])
    %     ylim([0 35])
    %
    %     waitfor(vizRate);
end


end