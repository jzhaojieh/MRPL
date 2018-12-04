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

RobotSystem.executeTrajectoryToPose(0.9144, 0.3048, 0, 1);
RobotSystem.pid.actualPoses(end) = RobotSystem.pid.fusionPoses(end);

RobotSystem.executeTrajectoryToPose(0.6096, 0.6096, pi()/2.0, 1);
RobotSystem.pid.actualPoses(end) = RobotSystem.pid.fusionPoses(end);

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