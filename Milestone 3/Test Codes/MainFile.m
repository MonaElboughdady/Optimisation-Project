Robot.number = 4;
Robot.wheelRadius = 0.1;
Robot.trackWidth = 1;
Robot.originalRobotKinematics = differentialDriveKinematics("WheelRadius",Robot.wheelRadius,"TrackWidth",Robot.trackWidth,"VehicleInputs","VehicleSpeedHeadingRate");
Robot.robots = repelem([Robot.originalRobotKinematics.copy()],Robot.number);
Robot.initOrientation = (pi/4)*ones(Robot.number,1);
Robot.initPosition = initializePositions(Robot.trackWidth*2,Robot.number);
Robot.initPose = [Robot.initPosition Robot.initOrientation]';
Robot.pathsColors = rand(Robot.number,3);


Controller.desiredLinearVelocity = 3;
Controller.maxAngularVelocity = 2;
Controller.lookaheadDistance = 0.5;
Controller.controllers = cell(1,Robot.number);
for i = 1:Robot.number
    Controller.controllers{1,i} = controllerPurePursuit("DesiredLinearVelocity",Controller.desiredLinearVelocity,"MaxAngularVelocity",Controller.maxAngularVelocity,"LookaheadDistance",Controller.lookaheadDistance);
end
Controller.sampleTime = 0.1;


Map.goalCenter = [30 30];
Map.numberofPathsPoints = 4;
Map.goals = Robot.initPosition+Map.goalCenter;
Map.waypointRadius = 10;
Map.goalRadius = 2;
Map.size = [100 100];
Map.numberofObstacles = 3;
Map.radiusofObstacles = [5 7 20];
Map.locationofObstacles = [30 37 30
                           30 38 50];



% for i = 1:Robot.number
%     Controller.controllers{1,i}.Waypoints = [Robot.initPosition(i,:)
%                                            10+4*i 10+4*i
%                                            20+4*i 15+4*i
%                                            Map.goals(i,:)];                                   
%      
% end
% 
%        
% [pos,vel] = simulateKinematics(Robot,Controller,Map,true)
runSA
