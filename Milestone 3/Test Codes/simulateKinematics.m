function [posArray,velArray] = simulateKinematics(Robot,Controller,Map,visualize)
disp("Entered Simulation")
tic;
goalRadius = Map.goalRadius;
sampleTime = Controller.sampleTime;
distanceToGoal = cellfun(@norm,num2cell(Robot.initPosition - Map.goals,2));
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
            continue
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
        xlim([0 40])
        ylim([0 40])
        waitfor(vizRate);    
    end
    
end
toc
end