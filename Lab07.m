%%%%%%%%%%%%LAB-07%%%%%%%%%%%%

clear;
clear classes;

%%%%%%%Setup mrplSystem%%%%%%%

sim =       0  ;
plot =      1  ;
feedback =  0  ;

global RobotSystem
RobotSystem = mrplSystem(sim, plot, feedback);
RobotSystem.robot.encoders.NewMessageFcn=@encoderEventListener;

%%%%%%Setup Trajectories%%%%%%

xf1 = 0.3048; yf1 = 0.3048; thf1 = 0.0;
xf2 = -0.3048; yf2 = -0.3048; thf2 = -pi()/2.0;
xf3 = 0; yf3 = 0; thf3 = 0;

%%%%%Execute Trajectories%%%%%

RobotSystem.executeTrajectoryToPose(xf1, yf1, thf1, 1);
RobotSystem.executeTrajectoryToPose(xf2, yf2, thf2, 1);
RobotSystem.executeTrajectoryToPose(xf3, yf3, thf3, 1);

%%%%%%%%Shutdown Robot%%%%%%%%

RobotSystem.robot.stop();
RobotSystem.robot.shutdown();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function encoderEventListener(handle, event)
    global RobotSystem
    if RobotSystem.tstamp == 0
        RobotSystem.tstamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1e9;
    end
    RobotSystem.encoderTimeStamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1e9;
    e = [RobotSystem.robot.encoders.LatestMessage.Vector.X, RobotSystem.robot.encoders.LatestMessage.Vector.Y];
    RobotSystem.newenc = e;
end