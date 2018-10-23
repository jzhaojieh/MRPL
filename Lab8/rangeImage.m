classdef rangeImage < handle
    properties(Constant)
        maxUsefulRange = 2.0;
        minUsefulRange = 0.05;
        maxRangeForTarget = 1.0;
        sailDistance = 0.127;
        threshold = 0.01;
    end
    properties(Access = public)
        rArray = [];
        tArray = [];
        xArray = [];
        yArray = [];
        numPix;
        indices = [];
        th = [];
    end
    methods(Access = public)
        function obj = rangeImage(ranges,skip,cleanFlag)
            % Constructs a rangeImage for the supplied data.
            % Converts the data to rectangular coordinates
            if(nargin == 3)
                n=0;
                for i=1:skip:length(ranges)
                    n = n + 1;
                    obj.rArray(n) = ranges(i);
                    obj.tArray(n) = (i-1)*(pi/180);
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

         function [err num th] = findLineCandidate(obj,middle,maxLen)
         % Find the longest sequence of pixels centered at pixel
         % ?middle? whose endpoints are separated by a length less
         % than the provided maximum. Return the line fit error, the
         % number of pixels participating, and the angle of
         % the line relative to the sensor.
         expectedPts = 8;
         pointSetX = zeros(expectedPts); pointSetY = zeros(expectedPts);
         pointSetX(expectedPts/2) = obj.xArray(middle);
         pointSetY(expectedPts/2) = obj.yArray(middle);
         decrement = middle;
         increment = middle;
         for i = 1:(expectedPts/2 - 1)
             obj.inc(increment);
             obj.dec(decrement);
             pointSetX(expectedPts/2 + i) = obj.xArray(increment);
             pointSetY(expectedPts/2 + i) = obj.xArray(increment);
             pointSetX(expectedPts/2 - i) = obj.xArray(decrement);
             pointSetY(expectedPts/2 - i) = obj.xArray(decrement);
         end
         numPoints = length(pointSetX);
         centroidX = mean(pointSetX);
         centroidY = mean(pointSetY);
         psXC = pointSetX - centroidX;
         psYC = pointSetY - centroidY;
         Ixx = sum(psXC.^2);
         Iyy = sum(psYC.^2);
         Ixy = sum(-1*(psXC.*psYC));
         Inertia = [Ixx, Ixy;Ixy, Iyy] / numPoints;
         lambda = eig(Inertia);
         lambda = sqrt(lambda)*1000.0;
            leftX = min(pointSetX);
            rightX = max(pointSetX);
            topY = max(pointSetY);
            botY = min(pointSetY);
            diag = sqrt((rightX - leftX)^2 + (botY - topY)^2);
            th = atan2(2*Ixy,Iyy-Ixx)/2.0;
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