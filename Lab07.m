%%%%%%%%%%%%LAB-07%%%%%%%%%%%%

clear;
clear classes;

%%%%%%%Setup mrplSystem%%%%%%%

sim =       0  ;
plot =      1  ;
feedback =  1  ;

RobotSystem = mrplSystem(sim, plot, feedback);

%%%%%%Setup Trajectories%%%%%%

xf1 = 0.3048; yf1 = 0.3048; thf1 = 0.0;
xf2 = -0.6096; yf2 = -0.6096; thf2 = -pi()/2.0;
xf3 = -0.3048; yf3 = 0.3048; thf3 = pi()/2.0;

%%%%%Execute Trajectories%%%%%

RobotSystem.executeTrajectoryRelativeToPose(xf1, yf1, thf1, 1);
RobotSystem.executeTrajectoryRelativeToPose(xf2, yf2, thf2, 1);
RobotSystem.executeTrajectoryRelativeToPose(xf3, yf3, thf3, 1);

disp([RobotSystem.pid.actualPoses(end).x, RobotSystem.pid.actualPoses(end).y, RobotSystem.pid.actualPoses(end).th]);

%%%%%%%%Shutdown Robot%%%%%%%%

RobotSystem.robot.stop();
RobotSystem.robot.shutdown();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%