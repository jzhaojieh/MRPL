%% Lab 1 Task 1 Move the Robot
robot = raspbot();
robot.startLaser();

pause(1);

for c = 0:100
    lidarData = robot.laser.LatestMessage.Ranges;
    %limits range of sensor
    for i = 1:96
        if lidarData(i) > 1 || lidarData(i) < .06
            lidarData(i) = 1;
        end
    end
    for i = 276:360
        if lidarData(i) > 1 || lidarData(i) < .06
            lidarData(i) = 1;
        end
    end
    
    %index of min range
    minI = 6;
    %min range
    minR = 1;
    for i = 1:360
        if minR > lidarData(i)
            minI = i;
            minR = lidarData(i);
        end
    end
    %get x y of closest object
    [minX, minY, bearing] = irToXy(minI, minR);
    
    %plot the closest object
    plot([minX, 0], [minY, 0], 'x'); 
        axis([-1 1 -1 1]); %for rescaling
    %velocity proportional to distance
    v = (minR - .5)*.3;
    %.044 is distance from wheel to center of robot
    if v >=0
        vR = v + .044*v*(bearing/minR);
        vL = v - .044*v*(bearing/minR);
    else %robot is backing away and tries to keep facing object so signs are switched
        vR = v - .044*v*(bearing/minR);
        vL = v + .044*v*(bearing/minR);
    end
    robot.sendVelocity(vL, vR)
    pause(.2);
end

robot.stop();
robot.stopLaser();
robot.shutdown();

function [x, y, th] = irToXy(i, r)
    if i < 186
        th = (i-6)*(pi/180);
    else
        th = (i-366)*(pi/180);
    end
    x = r*cos(th);
    y = r*sin(th);
end