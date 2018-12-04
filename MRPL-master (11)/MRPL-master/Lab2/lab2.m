%% Lab 2 Task 1 Read and Display Lidar Data
% robot = raspbot('sim');
global data;
data = [];
robot = raspbot('Raspbot-03');
robot.startLaser();
pause(3);
follow(robot)
robot.stopLaser();

function [x, y, b] = irToxy(i, r)
    t1 = -5.0*pi/180;
    b = t1 + (i-1) * pi/180;
     if b > pi
         b = b - 2*pi;
     end
    x = r * cos(b);
    y = r * sin(b);
end

function [x, y, b, d] = find(robot)
    d2 = robot.laser.LatestMessage.Ranges;
    d = 1.0;
    x = 0;
    y = 0;
    b = 0;
    for i = 1:360
        if (d2(i) >= 0.06 && d2(i) <= d)
            [x1, y1, b1] = irToxy(i, d2(i));
%             data = [data, [i]];
            if abs(b1) < (pi/2)
                x = x1;
                y = y1;
                b = b1;
                d = d2(i);
%             plot(x, y, 'x')
            end
        end
    end
end

function [] = follow(robot)
    idealRange = 0.5;
    gain = .2;
    figure(1);
    clf;
    for i = 0:50
        [x, y, b, d] = find(robot);
        vel = (d - idealRange) * gain;
        vr = vel + .044 * (y/d^2);
        vl = vel - .044 * (y/d^2);
        robot.sendVelocity(vl, vr)
        pause(0.5)
        plot(-y, x, 'x')
        axis([-2 2 -2 2]);
        title('Closest point in robot frame');
        xlabel('X (m)');
        ylabel('Y (m)');
    end
end



