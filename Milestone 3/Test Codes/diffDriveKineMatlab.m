robot = differentialDriveKinematics("WheelRadius",0.1,"TrackWidth",1,"VehicleInputs","VehicleSpeedHeadingRate");
robot1 = robot.copy();
initOrientation = 0;
currPos = [0 0];
currPos1 = [0 5];
path = [0 0;
        10 10;
        20 10;
        30 30];
% plot(path(:,1),path(:,2),'k--d')
xlim([0 35])
ylim([0 35])
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 5;
controller.MaxAngularVelocity = 3;
controller.LookaheadDistance = 0.5;
goalRadius = 5;
robotGoal = [30 30];
robotInitialLocation = [currPos initOrientation]';
robotInitialLocation1 = [currPos1 initOrientation]';
robotCurrentPose = robotInitialLocation;
robotCurrentPose1 = robotInitialLocation1;
distanceToGoal = norm(robotInitialLocation - robotGoal);
velarray = [];
poseArray = [];
sampleTime = 0.01;
vizRate = rateControl(1/sampleTime);
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    [v1, omega1] = controller(robotCurrentPose1);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    vel1 = derivative(robot1, robotCurrentPose1, [v1 omega1]);
    velarray = [velarray vel];
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    robotCurrentPose1 = robotCurrentPose1 + vel1*sampleTime; 
    poseArray = [poseArray robotCurrentPose]; 
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    distanceToGoal1 = norm(robotCurrentPose1(1:2) - robotGoal(:));
    
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotTrVec1 = [robotCurrentPose1(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotRot1 = axang2quat([0 0 1 robotCurrentPose1(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    plotTransforms(plotTrVec1', plotRot1, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 35])
    ylim([0 35])
    
    waitfor(vizRate);
end