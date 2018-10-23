%%%%%%%%%%%%LAB-08%%%%%%%%%%%%

clear;
clear classes;

%%%%%%%Setup mrplSystem%%%%%%%

sim =       0  ;
plot =      1  ;
feedback =  1  ;
skip =      1  ;
cleanFlag = 0  ;

global RobotSystem
RobotSystem = mrplSystem(sim, plot, feedback);
RobotSystem.robot.startLaser();
RobotSystem.robot.encoders.NewMessageFcn=@encoderEventListener;

%%%%%%%%%Laser Stuff%%%%%%%%%

pause(4);
ranges = RobotSystem.robot.laser.LatestMessage.Ranges;           
image = rangeImage(ranges,skip,cleanFlag);
[centroidX, centroidY, th] = image.findLineCandidate();
disp([centroidX, centroidY, th]);
RobotSystem.executeTrajectoryToPose(centroidX-.2, centroidY, th, 1);
ranges = RobotSystem.robot.laser.LatestMessage.Ranges;           
image = rangeImage(ranges,skip,cleanFlag);
[centroidX, centroidY, th] = image.findLineCandidate();
disp([centroidX, centroidY, th]);
RobotSystem.executeTrajectoryRelativeToPose(centroidX+.02, centroidY, th, 1);
RobotSystem.robot.forksUp();
pause(2);
RobotSystem.robot.forksDown();

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