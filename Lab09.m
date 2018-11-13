%%%%%%%%%%%%LAB-08%%%%%%%%%%%%

clear;
clear classes;

%%%%%%%Setup Constants%%%%%%%

sim =        1    ;
plot =       0    ;
feedback =   1    ;
cleanFlag =  0    ;
skip =       1    ;
leftIndex =  275  ;
rightIndex = 95   ;

%%%%%%%Setup mrplSystem%%%%%%%

global RobotSystem
RobotSystem = mrplSystem(sim, plot, feedback);
RobotSystem.robot.startLaser();
RobotSystem.robot.encoders.NewMessageFcn=@encoderEventListener;
RobotSystem.robot.forksDown();

pause(4);

%%%%%%%%%Laser Stuff%%%%%%%%%
for i = 1:3
    
    %%%%Wait until it picks up a pallet becuase it can be unreliable%%%%
    isZero = 1;
    while isZero == 1
        ranges = RobotSystem.robot.laser.LatestMessage.Ranges; 
        %plot(1:360, ranges);
        image = rangeImage(ranges,skip,cleanFlag);
        [isZero, centroidX, centroidY, th] = image.getPalletLoc(RobotSystem, leftIndex, rightIndex);
        pause(.1);
    end
    
    %%%%execute trajectory to get closer to the pallet%%%%
    RobotSystem.executeTrajectoryToPose(centroidX + sign(centroidX) * (-.2), centroidY, th, 1);
    
    %%%%Wait until it picks up a pallet becuase it can be unreliable%%%%
    isZero = 1;
    while isZero == 1
        ranges = RobotSystem.robot.laser.LatestMessage.Ranges;
        %plot(1:360, ranges);
        image = rangeImage(ranges,skip,cleanFlag);
        [isZero, centroidX, centroidY, th] = image.getPalletLoc(RobotSystem, leftIndex, rightIndex);
        pause(.1);
    end
    
    %%%%execute trajectory to pick up the pallet%%%%
    RobotSystem.executeTrajectoryToPose(centroidX + sign(centroidX) * (-.03), centroidY, th, 1);
    
    %%%%Pick up pallet, wait, put it down again%%%%
    RobotSystem.robot.forksUp();
    pause(1);
    RobotSystem.robot.forksDown();
    pause(1);
    
    %%%%Back up and turn around%%%%
    RobotSystem.moveRelDist(-.1);
    RobotSystem.turnRelAngle(180);
    
    %%%%Trying to do a thing where it returns to origin between pick-ups%%%%
    %RobotSystem.executeTrajectoryToPose(0, 0, pi*(abs(RobotSystem.pid.actualPoses(end).th)>(pi/2)), 1);
    
    %%%%Wait 10 seconds if not a simulator (I got sick of waiting for the simulator%%%%
    pause(1);
    if sim == 0
        pause(9);
    end
    
end

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