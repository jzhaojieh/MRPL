classdef rangeImage < handle
    properties(Constant)
        maxUsefulRange = 2.0;
        minUsefulRange = 0.05;
        maxRangeForTarget = 1.0;
        sailDistance = 0.127;
        threshold = 0.01;
        maxDist = 1.5;
        Iyythreshold = 1;
        fitErrorThreshhold = .005;
    end
    properties(Access = public)
        rArray = [];
        tArray = [];
        xArray = [];
        yArray = [];
        numPix;
        indices = [];
        th = [];
        robotModel;
    end
    methods(Access = public)
        function obj = rangeImage(ranges,skip,cleanFlag)
            % Constructs a rangeImage for the supplied data.
            % Converts the data to rectangular coordinates
            obj.robotModel = robotModel();
            n = 0;
            if(nargin == 3)
                for i=1:skip:length(ranges)
                    n = n + 1;
                    obj.rArray(n) = ranges(i);
                    obj.tArray(n) = (i-6)*(pi/180);
                    obj.xArray(n) = ranges(i)*cos(obj.tArray(n));
                    obj.yArray(n) = ranges(i)*sin(obj.tArray(n));
                end
                obj.numPix = n;
                if cleanFlag; obj.removeBadPoints(); end
            end
        end
        function removeBadPoints(obj)
        % takes all points above and below two range thresholds
        % out of the arrays. This is a convenience but the result
        % should not be used by any routine that expects the points
        % to be equally separated in angle. The operation is done
        % inline and removed data is deleted.
            goodOnes = obj.rArray > 0.06 & obj.rArray < 4.0;
            obj.rArray = obj.rArray(goodOnes);
            obj.indices = linspace(2,obj.numPix,obj.numPix);
            obj.indices = obj.indices(goodOnes);
            % Compute the angles of surviving points
            obj.th = (obj.indices-1)*(pi/180) - atan2(0.024,0.28);
         end

         function plotRvsTh(obj, maxRange)
         % plot the range image after removing all points exceeding
         % maxRange
            %% FILL ME IN
         end

         function plotXvsY(obj, maxRange)
         % plot the range image after removing all points exceeding
         % maxRange
            %% FILL ME IN
         end

         function [isSail, th, numPoints] = findLineCandidate(obj, midpoint)
         % Find the longest sequence of pixels centered at pixel
         % ?middle? whose endpoints are separated by a length less
         % than the provided maximum. Return the line fit error, the
         % number of pixels participating, and the angle of
         % the line relative to the sensor.
        
        oneSide = false;
        
        th = 0;
        lineFound = false;
        pointSetX = []; pointSetY = [];
        x = obj.xArray(midpoint); y = obj.yArray(midpoint);
        pointSetX = [pointSetX x];
        pointSetY = [pointSetY y];
        leftIndex = midpoint;
        rightIndex = midpoint;
        sailDist = obj.rArray(midpoint);
        
        prevLambda = [0,0];
        i = 1;
        while lineFound == false
            %%%%This if statement alternates adding a point to the left and adding a point to the right%%%%
            if mod(i, 2) == 1
                leftIndex = obj.dec(leftIndex);
                leftX = obj.xArray(leftIndex); leftY = obj.yArray(leftIndex);
                rightX = obj.xArray(rightIndex); rightY = obj.yArray(rightIndex);
                sailLength = sqrt((rightX - leftX)^2 + (rightY - leftY)^2);
                
                pointSetX = [leftX, pointSetX];
                pointSetY = [leftY, pointSetY];
            else
                rightIndex = obj.inc(rightIndex);
                leftX = obj.xArray(leftIndex); leftY = obj.yArray(leftIndex);
                rightX = obj.xArray(rightIndex); rightY = obj.yArray(rightIndex);
                sailLength = sqrt((rightX - leftX)^2 + (rightY - leftY)^2);
                
                pointSetX = [pointSetX, rightX];
                pointSetY = [pointSetY, rightY];
            end
            
            %%%%Calculates moment and eigenvalues%%%%
            numPoints = length(pointSetX);
            psXC = pointSetX - x;
            psYC = pointSetY - y;
            Ixx = sum(psXC.^2);
            Iyy = sum(psYC.^2);
            Ixy = sum(-1*(psXC.*psYC));
            Inertia = [Ixx Ixy;Ixy Iyy] / numPoints;
            lambda = eig(Inertia);
            lambda = sqrt(lambda)*1000.0;
            %%%%If the set of points has a length greater than a sail%%%%
            if sailLength > obj.sailDistance && ~oneSide
                oneSide = true;
                if mod(i, 2) == 1
                    pointSetX(1) = [];
                    pointSetY(1) = [];
                else
                    pointSetX(end) = [];
                    pointSetY(end) = [];
                end
                lambda = prevLambda;
            elseif sailLength > obj.sailDistance
                
                %%%%Remove the points just added%%%%
                if mod(i, 2) == 1
                    pointSetX(1) = [];
                    pointSetY(1) = [];
                else
                    pointSetX(end) = [];
                    pointSetY(end) = [];
                end
                
                %%%%Computes change in the eigenvalues to see if it is a wall%%%%
                if abs(prevLambda(2) - lambda(2)) > obj.Iyythreshold && length(pointSetX) > 3
                    %%%%is probably not a wall%%%%
                    
                    %%%%Gets a linear fit to the points to see if it is actually a line%%%%
                    [p, S] = polyfit(pointSetX, pointSetY, 1);
                    [y_fit, delta] = polyval(p, pointSetY, S);
                    fitError = mean(delta)/sailDist;
                    
                    %%%%recalculate the values for only the sail%%%%
                    numPoints = length(pointSetX);
                    psXC = pointSetX - x;
                    psYC = pointSetY - y;
                    Ixx = sum(psXC.^2);
                    Iyy = sum(psYC.^2);
                    Ixy = sum(-1*(psXC.*psYC));
                    Inertia = [Ixx Ixy;Ixy Iyy] / numPoints;
                    lambda = eig(Inertia);
                    lambda = sqrt(lambda)*1000.0;
                    sailLength = sqrt((pointSetX(end) - pointSetX(1))^2 + (pointSetY(end) - pointSetY(1))^2);
                    %%%%Checks data against a lot of constants to see if it should be counted as a valid sail%%%%
                    if ((numPoints >= 3) && (sailDist < obj.maxDist) && (sailLength < obj.sailDistance + obj.threshold) && abs(lambda(2)) < 50)
                        isSail = true;
                        th = atan2(2*Ixy,Iyy-Ixx)/2.0;
                        return
                    else
                        isSail = false;
                        return
                    end
                else
                    %%%%This set of points describes not a sail%%%%
                    isSail = false;
                    return
                end
            elseif oneSide
                oneSide = false;
                lambda = prevLambda;
                isSail = false;
                return
            end
            prevLambda = lambda;
            i = i + 1;
         end
         end
         
         function [isZero, centerX, centerY, centerTh] = getPalletLoc(obj, RobotSystem, leftIndex, rightIndex)
             %%%%go from leftIndex to rightIndex and see if any of the points correspond to a pallet%%%%
             midpoint = leftIndex+10;
             X = 0;
             Y = 0;
             Th = 0;
             palletPoints = 0;
             palletDist = 10;
             %%%%Loops from left to right through the range specified%%%%
             while midpoint ~= rightIndex-10
                 [isSail, testTh, numPoints] = obj.findLineCandidate(midpoint);
                 if(isSail && numPoints > palletPoints && obj.rArray(midpoint) <= palletDist && (obj.rArray(midpoint) > .054 || obj.rArray(midpoint) < .052))
                     palletPoints = numPoints;
                     X = obj.xArray(midpoint);
                     Y = obj.yArray(midpoint);
                     Th = testTh;
                     palletDist = obj.rArray(midpoint);
                     if X < .05
                         Th = 0;
                     end
                 end
                 midpoint = obj.inc(midpoint);
             end
             
             robotPose = RobotSystem.pid.actualPoses(end);
             
             goal = pose(X, Y, 0);
             convertedGoal = pose.matToPoseVec(robotPose.bToA() * goal.bToA());
             
             centerX = convertedGoal(1);
             centerY = convertedGoal(2);
             centerTh = convertedGoal(3);
             if abs(centerTh) > pi
                 centerTh = sign(centerTh)*(-1)*(2*pi-abs(centerTh));
             end
             
             isZero = 0;
             
             
             if X == 0 && Y == 0 && Th == 0
                 isZero = 1;
             end


             %%%%[Detected coordinates, robot coordinaates, converted coordinates]%%%%
             disp([X, Y, Th, 0, RobotSystem.idealPoses(end).x, RobotSystem.idealPoses(end).y, RobotSystem.idealPoses(end).th, 0, robotPose.x, robotPose.y, robotPose.th, 0, centerX, centerY, centerTh]);
         end

         function num = numPixels(obj)
         num = obj.numPix;
         end

         % Modulo arithmetic on nonnegative integers. MATLABs choice to
         % have matrix indices start at 1 has implications for
         % calculations relating to the position of a number in the
         % matrix. In particular, if you want an operation defined on
         % the sequence of numbers 1 2 3 4 that wraps around, the
         % traditional modulus operations will not work correctly.
         % The trick is to convert the index to 0 1 2 3 4, do the
         % math, and convert back.

         function out = inc(obj,in)
         % increment with wraparound over natural numbers
            out = obj.indexAdd(in,1);
         end

         function out = dec(obj,in)
         % decrement with wraparound over natural numbers
            out = obj.indexAdd(in,-1);
         end

         function out = indexAdd(obj,a,b)
         % add with wraparound over natural numbers. First number
         % ?a? is "natural" meaning it >=1. Second number is signed.
         % Convert a to 0:3 and add b (which is already 0:3).
         % Convert the result back by adding 1.
            out = mod((a-1)+b,obj.numPix)+1;
         end
     end
end