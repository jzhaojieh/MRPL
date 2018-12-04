%%%%%%%%%%%%LAB-08%%%%%%%%%%%%

clear;
clear classes;

%%%%%%%Setup Constants%%%%%%%

sim =        0    ;
plot =       1    ;
feedback =   1    ;
cleanFlag =  0    ;
skip =       1    ;
leftIndex =  275+50  ;
rightIndex = 95-50   ;

%%%%%%%Setup mrplSystem%%%%%%%

global RobotSystem
RobotSystem = mrplSystem(sim, plot, feedback);
RobotSystem.robot.startLaser();
RobotSystem.robot.encoders.NewMessageFcn=@encoderEventListener;
pause(4);

%%%%%%Setup Map Localizer%%%%%%

%%%%THIS IS WRONG AS FUCK%%%%
p1 = [-2 ; -2];
p2 = [ 2 ; -2];
p3 = [ 2 ; 2];
p4 = [-2 ; 2];
lines_p1 = [p1 p2 p3 p4];
lines_p2 = [p2 p3 p4 p1];
l = lineMapLocalizer(lines_p1, lines_p2, 0.3, 0.01, 0.0005);

%%%%Preset the robot's pose%%%%

RobotSystem.pid.actualPoses(end) = pose(0.6096,0.6096,pi()/2.0);
RobotSystem.pid.actualXs(end) = 0.6096;
RobotSystem.pid.actualYs(end) = 0.6096;
RobotSystem.pid.thArr(end) = pi()/2.0;
RobotSystem.pid.fusionPoses(end) = pose(0.6096,0.6096,pi()/2.0);
RobotSystem.idealPoses(end) = pose(0.6096,0.6096,pi()/2.0);

%%%%%%%%%Lab Run Stuff%%%%%%%%%

RobotSystem.executeTrajectoryToPose(0.3048, 0.9144, pi()/2.0, 1);
RobotSystem.pid.actualPoses(end) = RobotSystem.pid.fusionPoses(end);
%%%%Localize here based on new lidar data%%%%

RobotSystem.executeTrajectoryToPose(0.9144, 0.3048, 0, 1);
RobotSystem.pid.actualPoses(end) = RobotSystem.pid.fusionPoses(end);
%%%%Localize here based on new lidar data%%%%

RobotSystem.executeTrajectoryToPose(0.6096, 0.6096, pi()/2.0, 1);
RobotSystem.pid.actualPoses(end) = RobotSystem.pid.fusionPoses(end);
%%%%Localize here based on new lidar data%%%%

%%%%%%%%Shutdown Robot%%%%%%%%

RobotSystem.robot.stopLaser();
RobotSystem.robot.stop();
RobotSystem.robot.shutdown();

%%%%%%%Encoder Listener%%%%%%%

function encoderEventListener(handle, event)
    global RobotSystem
    if RobotSystem.tstamp == 0
        RobotSystem.tstamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1e9;
    end
    RobotSystem.encoderTimeStamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1e9;
    e = [RobotSystem.robot.encoders.LatestMessage.Vector.X, RobotSystem.robot.encoders.LatestMessage.Vector.Y];
    RobotSystem.newenc = e;
end