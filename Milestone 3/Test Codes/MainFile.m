%This is the main file code to run the Simulated Annealing (SA)
%optimization algorithm in solving the problem of Multi robot path planning
%for dynamic formation application previously "Multi robot flocking control"
%several tunable parameters can be changed to check their effect.
%By: Team 9 (Ahmed Zaghloul, Abdelrahamn Saad, Bishoy Atef, John Gameel, Mona Farouk)
%Date: Thursday, 11th November, 2021
%=========================================================================

%cleaning up the working environment before starting
clc, clear, close all

%Initialize Robot parameters:
Robot.number = 4; %number of mobile robots    
Robot.wheelRadius = 0.1; %radius of the robot wheel
Robot.trackWidth = 1; %track width of the robot
Robot.originalRobotKinematics = differentialDriveKinematics("WheelRadius"...
    ,Robot.wheelRadius,"TrackWidth",Robot.trackWidth,"VehicleInputs",...
    "VehicleSpeedHeadingRate"); %defining the kinematics of each robot 
Robot.robots = repelem([Robot.originalRobotKinematics.copy()],Robot.number);
%copying kinematics to each robot
Robot.initOrientation = (pi/4)*ones(Robot.number,1); %Initialize the orientation of the robot
Robot.initPosition = initializePositions(Robot.trackWidth*2,Robot.number); %Initialize the position of each robot
Robot.initPose = [Robot.initPosition Robot.initOrientation]'; %Creating the pose of each robot
Robot.pathsColors = rand(Robot.number,3); %Identify the color of each robot path

%Setting controller parameters:
Controller.desiredLinearVelocity = 3; %Set the desired linear velocity of the robots
Controller.maxAngularVelocity = 2;  %Set the max angular velocity of the robots
Controller.lookaheadDistance = 0.5; %Set the lookahead distance of the robots
Controller.controllers = cell(1,Robot.number); %creating a cell array with the controllers


for i = 1:Robot.number %looping on each robot to create the controller of each robot 
    Controller.controllers{1,i} = controllerPurePursuit("DesiredLinearVelocity",...
        Controller.desiredLinearVelocity,"MaxAngularVelocity",Controller.maxAngularVelocity,...
        "LookaheadDistance",Controller.lookaheadDistance);
end
Controller.sampleTime = 0.1; %Setting the sample time of the controller

%Initialize Map parameters:
Map.goalCenter = [30 30]; %the coordinates of the goal center       
Map.numberofPathsPoints = 4; %the number of points in the path
Map.goals = Robot.initPosition + Map.goalCenter; %final destination of each robot
Map.waypointRadius = 10; %Number of waypoints
Map.goalRadius = 2; %Radius of the goal
Map.size = [100 100]; % Xmax and Ymax of the map
Map.numberofObstacles = 3; %Number of obstacles
Map.radiusofObstacles = [5 7 20]; %Different Radii for obstacles 
Map.locationofObstacles = [30 37 30 
                           30 38 50]; % X and Y of each obstacle

runSA