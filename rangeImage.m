classdef rangeImage < handle
    properties(Constant)
        maxUsefulRange = 2.0;
        minUsefulRange = 0.05;
        maxRangeForTarget = 1.0;
        sailDistance = 0.127;
        threshold = 0.01;
        maxDist = 1;
        Iyythreshold = 5;
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
         
        % Compute the angles of surviving points
        
        th = 0;
        lineFound = false;
        pointSetX = []; pointSetY = [];
        x = obj.xArray(midpoint); y = obj.yArray(midpoint);
        pointSetX = [pointSetX x];
        pointSetY = [pointSetY y];
        leftIndex = midpoint;
        rightIndex = midpoint;
        sailDist = obj.rArray(midpoint);
        
        prevLambda = 0;
        
        while lineFound == false
            leftIndex = obj.dec(leftIndex);
            rightIndex = obj.inc(rightIndex);
            leftX = obj.xArray(leftIndex); leftY = obj.yArray(leftIndex);
            rightX = obj.xArray(rightIndex); rightY = obj.yArray(rightIndex);
            sailLength = sqrt((rightX - leftX)^2 + (rightY - leftY)^2);
            
            pointSetX = [leftX, pointSetX, rightX];
            pointSetY = [leftY, pointSetY, rightY];
            
            numPoints = length(pointSetX);
            psXC = pointSetX - x;
            psYC = pointSetY - y;
            Ixx = sum(psXC.^2);
            Iyy = sum(psYC.^2);
            Ixy = sum(-1*(psXC.*psYC));
            Inertia = [Ixx Ixy;Ixy Iyy] / numPoints;
            lambda = eig(Inertia);
            lambda = sqrt(lambda)*1000.0;
            
            if sailLength > obj.sailDistance + obj.threshold
                if abs(prevLambda(1) - lambda(1)) > obj.Iyythreshold && length(pointSetX) > 3
                    %is not a wall
                    pointSetX(1) = [];
                    pointSetX(end) = [];
                    pointSetY(1) = [];
                    pointSetY(end) = [];
                    %recalculate the lambda and stuff
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
                    [p, S] = polyfit(pointSetX, pointSetY, 1);
                    [y_fit, delta] = polyval(p, pointSetY, S);
                    fitError = mean(delta)/sailDist;
                    fitError = .1;
                    if ((numPoints >= 7) && (lambda(1) < 1.3) && (sailDist < obj.maxDist) && (sailDist ~= .053) && (sailLength < obj.sailDistance+obj.threshold) && (fitError > obj.fitErrorThreshhold))
                        isSail = true;
                        th = atan2(2*Ixy,Iyy-Ixx)/2.0;
                        return
                    else
                        isSail = false;
                        return
                    end
                else
                    %is a wall
                    isSail = false;
                    return
                end
            end
            prevLambda = lambda;
         end
         end
         
         function [centerX, centerY, centerTh] = getPalletLoc(obj, RobotSystem, leftIndex, rightIndex)
             %go from leftIndex to rightIndex and see if any of the points
             %correspond to a pallet
             midpoint = leftIndex;
             X = 0;
             Y = 0;
             Th = 0;
             palletPoints = 0;
             palletDist = 10;
             while midpoint ~= rightIndex
                 [isSail, testTh, numPoints] = obj.findLineCandidate(midpoint);
                 if(isSail && numPoints > palletPoints && obj.rArray(midpoint) <= palletDist + obj.threshold && (obj.rArray(midpoint) > .054 || obj.rArray(midpoint) < .052))
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
             
             %globalPose = pose.matToPoseVec(pose(X, Y, Th).bToA() * robotPose.bToA() * pose(0,0,0).aToB());
             %centerX = globalPose(1);
             %centerY = globalPose(2);
             %centerTh = globalPose(3);
             
             centerX = (robotPose.x + X * cos(robotPose.th));
             centerY = -1 * (robotPose.y + Y * sin(robotPose.th));
             centerTh = (robotPose.th + Th);
             centerTh = mod (centerTh, 2*pi);
             if centerTh > pi
                 centerTh = 2*pi-centerTh;
             end
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