%% Lab 1 Task 1 Move the Robot
robot = raspbot();
robot.startLaser();

pause(2);

for c = 0:200
    lidarData = robot.laser.LatestMessage.Ranges;
    %limits range of sensor
    for i = 1:360
        if lidarData(i) > 1 || lidarData(i) < .06
            lidarData(i) = 1;
        end
    end
    
    %index of min range
    minI = 6;
    %min range
    minR = 1;
    for i = 1:96
        if minR > lidarData(i)
            minI = i;
            minR = lidarData(i);
        end
    end
    for i = 276:360
        if minR > lidarData(i)
            minI = i;
            minR = lidarData(i);
        end
    end
    %get x y of closest object
    [minX, minY, bearing] = irToXy(minI, minR);
    
    %plot the closest object
    plot(minX, minY, 'x');
    xlabel("Distance in m");
    ylabel("Distance in m"); 
    title('Position of closest object relative to robot');
        axis([-2 2 -2 2]); %for rescaling
    %velocity proportional to distance
    v = (minR - .5)*.4;
    %.044 is distance from wheel to center of robot
    if v >= 0
        vR = v + .044*v*7*(minY/(minR^2));
        vL = v - .044*v*7*(minY/(minR^2));
    else
        vR = v - .044*v*3*(minY/(minR^2));
        vL = v + .044*v*3*(minY/(minR^2));
    end
    robot.sendVelocity(vL, vR)
    pause(.2);
end

robot.stop();
robot.stopLaser();
robot.shutdown();

function [x, y, th] = irToXy(i, r)
    th = (i-6)*(pi/180);
    if th > pi
        th = th - 2*pi;
    end
    x = r*cos(th);
    y = r*sin(th);
end