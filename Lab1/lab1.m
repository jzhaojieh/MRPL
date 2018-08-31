%% Lab 1 Task 1 Move the Robot
robot = raspbot('sim')
timeArray = [];
leftArray = [];
rightArray = [];
leftStart = robot.encoders.LatestMessage.Vector.Y;
rightStart = robot.encoders.LatestMessage.Vector.X;

start = tic;
signedDistance = 0;
while signedDistance < 0.3048
    robot.sendVelocity(.05,.05)
    lDist = abs(robot.encoders.LatestMessage.Vector.Y - leftStart);
	rDist = abs(robot.encoders.LatestMessage.Vector.X - rightStart);
    signedDistance = (lDist + rDist) / 2
    timeArray = [timeArray toc(start)];
    leftArray = [leftArray lDist];
    rightArray = [rightArray rDist];
    plot(timeArray,leftArray,timeArray,rightArray);
    pause(.05)
end
robot.sendVelocity(0,0)
pause(1)
while signedDistance > 0
    robot.sendVelocity(-.05, -.05)
    lDist = (robot.encoders.LatestMessage.Vector.Y - leftStart);
	rDist = (robot.encoders.LatestMessage.Vector.X - rightStart);
    signedDistance = (lDist + rDist) / 2
    timeArray = [timeArray toc(start)];
    leftArray = [leftArray lDist];
    rightArray = [rightArray rDist];
    plot(timeArray,leftArray,timeArray,rightArray);
    pause(.05)
end
robot.sendVelocity(0,0)
robot.shutdown()

%% code
error('Mid script reached, this error message was made on purpose')