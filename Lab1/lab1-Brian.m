%% Lab 1 Task 1 Move the Robot
robot = raspbot();

v = .05

timeArray = []
leftArray = []
rightArray = []

leftStart = robot.encoders.LatestMessage.Vector.X
rightStart = robot.encoders.LatestMessage.Vector.Y

signedDistance = 0

startTime = tic

while signedDistance < .305
    
    robot.sendVelocity( v, v)
    pause(.05)
    
    leftEncoder = robot.encoders.LatestMessage.Vector.X
    rightEncoder = robot.encoders.LatestMessage.Vector.Y
    signedDistance = mean([leftEncoder - leftStart, rightEncoder - rightStart])
    
    leftArray = [leftArray, (leftEncoder - leftStart)*100]
    rightArray = [rightArray, (rightEncoder - rightStart)*100]
    timeArray = [timeArray, toc(startTime)]
    
    plot(timeArray,leftArray,timeArray,rightArray)
    
end

pause(.05)
robot.sendVelocity(0,0)
pause(1)

while signedDistance > 0
    
    robot.sendVelocity( -v, -v)
    pause(.05)
    
    leftEncoder = robot.encoders.LatestMessage.Vector.X
    rightEncoder = robot.encoders.LatestMessage.Vector.Y
    signedDistance = mean([leftEncoder - leftStart, rightEncoder - rightStart])
    
    leftArray = [leftArray, (leftEncoder - leftStart)*100]
    rightArray = [rightArray, (rightEncoder - rightStart)*100]
    timeArray = [timeArray, toc(startTime)]
    
    plot(timeArray,leftArray,timeArray,rightArray)
    
end

pause(.05)
robot.sendVelocity(0,0)
robot.shutdown

plot(timeArray,leftArray,timeArray,rightArray)

