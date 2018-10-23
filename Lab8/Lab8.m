robot = raspbot('Raspbot-07');

robot.startLaser();
pause(4);
ranges = robot.laser.LatestMessage.Ranges;           
xArr = []; yArr = [];
% remove all points with bad range
for i = 1:length(ranges) 
   [x1, y1, th1] = irToXy(i, ranges(i));
   xArr = [xArr x1]; yArr = [yArr y1];
end
findLineCandidate(ranges, xArr, yArr);
% scatter(yArr, xArr, 'g');
%obj.robot.stopLaser();
robot.stopLaser();
pause(1);
function findLineCandidate(rangeImg, xArr, yArr)
    goodOnes = rangeImg > 0.06 & rangeImg < 4.0;
    rangeImg = rangeImg(goodOnes);
    indices = linspace(2,length(rangeImg),length(rangeImg));
    indices = indices(goodOnes);
    % Compute the angles of surviving points
    th = (indices-1)*(pi/180) - atan2(0.024,0.28);
    for i = 1:length(rangeImg)
        pointSetX = []; pointSetY = [];
        x = xArr(i); y = yArr(i);
        for j = 1:length(rangeImg)
            otherX = xArr(j); otherY = yArr(j);
            dist = sqrt((otherX - x)^2 + (otherY - y)^2);
            threshold = 0.01;
            if dist <= 0.0635 + threshold;
                pointSetX = [pointSetX otherX];
                pointSetY = [pointSetY otherY];
            end
        end
        numPoints = length(pointSetX);
        centroidX = mean(pointSetX);
        centroidY = mean(pointSetY);
        dist = sqrt(centroidX^2 + centroidY^2);
        psXC = pointSetX - centroidX;
        psYC = pointSetY - centroidY;
        Ixx = sum(psXC.^2);
        Iyy = sum(psYC.^2);
        Ixy = sum(-1*(psXC.*psYC));
        Inertia = [Ixx Ixy;Ixy Iyy] / numPoints;
        lambda = eig(Inertia);
        lambda = sqrt(lambda)*1000.0;
        if ((numPoints >= 7) && (lambda(1) < 1.3) && dist < maxDist)
            leftX = min(pointSetX);
            rightX = max(pointSetX);
            topY = max(pointSetY);
            botY = min(pointSetY);
            diag = sqrt((rightX - leftX)^2 + (bottomY - topY)^2);
            if diag < .038
                th = atan2(2*Ixy,Iyy-Ixx)/2.0;
                plot([leftX, rightX], [topY, botY]);
                plot(centroidX, centroidY, 'r*');
                fprintf("orientation %d, centroidX %d, centroidY %d,\n", th, centroidX, centroidY);
            end
        end
    end
end
function [ x, y, th] = irToXy(i, r )
    t1 = -5.0*pi/180;
    t1 = atan2(0.024,0.28);
    th = t1 + (i-1) * pi/180;
     if th > pi
         th = th - 2*pi;
     end
    x = r * cos(th);
    y = r * sin(th);
end