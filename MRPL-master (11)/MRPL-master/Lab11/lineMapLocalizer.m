classdef lineMapLocalizer < handle
 %mapLocalizer A class to match a range scan against a map in
 % order to find the true location of the range scan relative to
 % the map.

     properties(Constant)
        maxErr = 0.01; % 5 cm
        minPts = 3; % min # of points that must match
     end

     properties(Access = private)
     end

     properties(Access = public)
         lines_p1 = [];
         lines_p2 = [];
         gain = .3;
         errThresh = 0.005;
         gradThresh = 0.0005;
         ptsInRangeImage;
     end
     
     methods(Static = true)
     
         function obj = lineMapLocalizer(lines_p1, lines_p2, errThresh, gradThresh)
             % create a lineMapLocalizer
             obj.lines_p1 = lines_p1;
             obj.lines_p2 = lines_p2;
             obj.errThresh = errThresh;
             obj.gradThresh = gradThresh;
         end
         
        function ro2 = closestSquaredDistanceToLines(obj,pi)
            % Find the squared shortest distance from pi to any line
            % segment in the supplied list of line segments.
            % pi is an array of 2d points
            % throw away homogenous flag
            pi = pi(1:2,:);
            
            r2Array = zeros(size(obj.lines_p1,2),size(pi,2));
 
            for i = 1:size(obj.lines_p1,2)
                [r2Array(i,:) , ~] = lineMapLocalizer.closestPointOnLineSegment(pi, obj.lines_p1(:,i), obj.lines_p2(:,i));
            end
            ro2 = min(r2Array,[],1);
        end
        
        
         function ids = throwOutliers(obj,pose,ptsInModelFrame)
        %Throws out data points that are endpoints or "stick out" too much
        %from the others
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(obj, worldPts);
            ids = find(r2 > lineMapLocalizer.maxErr*lineMapLocalizer.maxErr);
         end
        
         function avgErr2 = fitError(obj,pose,ptsInModelFrame)
        % Find the variance of perpendicular distances of
        % all points to all lines
        % transform the points
            worldPts = pose.bToA()*ptsInModelFrame;
        
            r2 = obj.closestSquaredDistanceToLines(obj, worldPts);
            r2(r2 == Inf) = [];
            err2 = sum(r2);
            num = length(r2);
            if(num >= lineMapLocalizer.minPts)
                avgErr2 = err2/num;
            else
            % not enough points to make a guess, so will not guess
                avgErr2 = inf;
            end
         end
        
        %[scalar, vector]
        function [err2_Plus0, J] = getJacobian(obj,poseIn,modelPts)
        % Computes the gradient of the error function
            err2_Plus0 = obj.fitError(obj, poseIn, modelPts);
            eps = 1e-5; %Tweak this??
            dp = [eps ; 0.0 ; 0.0];
            newPose = pose(poseIn.getPoseVec+dp);
%===== WRITTEN BY MERPLE SQUAD FROM HERE ====================================
            errorX = (obj.fitError(obj,newPose,modelPts) - err2_Plus0)/eps;
                        
            dp = [0.0 ; eps ; 0.0];
            newPose = pose(poseIn.getPoseVec+dp);
            errorY = (obj.fitError(obj,newPose,modelPts) - err2_Plus0)/eps;
            
            dp = [0.0 ; 0.0 ; eps];
            newPose = pose(poseIn.getPoseVec+dp);
            errorTh = (obj.fitError(obj,newPose,modelPts) - err2_Plus0)/eps;
            J = [errorX;errorY;errorTh];
        end
        
        %As is, the equations themselves of the jacobian and new poses should be working
        %Play with epsilon value in getJacobian??? Or Fix Tau in
        %controller? (Ideally, we should NOT touch the controlller.
        function [success, outPose] = refinePose(obj, inPose, ptsInModelFrame, maxIters)
         % refine robot pose in world (inPose) based on lidar
         % registration. Terminates if maxIters iterations is
         % exceeded or if insufficient points match the lines.
         % Even if the minimum is not found, outPose will contain 
         % any changes that reduced the fit error. Pose changes that
         % increase fit error are not included and termination
         % occurs thereafter.
            curPose = inPose;
            success = 0;
            ids = obj.throwOutliers(obj, inPose, ptsInModelFrame);
            ptsInModelFrame(:,ids) = [];
            for i = 0:maxIters
                %Gets current error and gradient descent jacobian
                [curErr, J] = obj.getJacobian(obj, curPose, ptsInModelFrame);
                gradMag = sqrt(sum(J .* J));
                %subtracts the change in error from the poses
                curPose = pose(curPose.getPoseVec - [obj.gain*gradMag*J(1); obj.gain*gradMag*J(2); obj.gain*gradMag*J(3)]);
                if (curErr < obj.errThresh)
                    disp("success");
                    outPose = curPose;
                    success = 1;
                    return;
                end
            end
            disp("failure");
            outPose = curPose;
        end
        
%================THIS WAS GIVEN============================================
        function [rad2, po] = closestPointOnLineSegment(pi,p1,p2)
            % Given set of points and a line segment, returns the
            % closest point and square of distance to segment for
            % each point. If the closest point is an endpoint, returns
            % infinity for rad2 because such points are bad for
            % lidar matching localization.
            %
            % [rad2, po] = CLOSESTPOINTONLINESEGMENT(pi,p1,p2)
            %
            % pi - Array of points of size 2 x n.
            % p1 - Column of size 2, endpoint of segment.
            % p2 - Column of size 2, endpoint of segment.
            %
            % rad2 - Squared distance to closest point on segment.
            % po - Closest points on segment. Same size as pi.
            v1 = bsxfun(@minus,pi,p1);
            v2 = p2-p1;
            v3 = bsxfun(@minus,pi,p2);
            v1dotv2 = bsxfun(@times,v1,v2);
            v1dotv2 = sum(v1dotv2,1);
            v2dotv2 = sum(v2.*v2);
            v3dotv2 = bsxfun(@times,v3,v2);
            v3dotv2 = sum(v3dotv2,1);
            nPoints = size(pi,2);
            rad2 = zeros(1,nPoints);
            po = zeros(2,nPoints);
            % Closest is on segment
            flag1 = v1dotv2 > 0.0 & v3dotv2 < 0.0;
            if any(flag1)
                scale = v1dotv2/v2dotv2;
                temp = bsxfun(@plus,v2*scale,[p1(1) ; p1(2)]);
                po(:,flag1) = temp(:,flag1);
                dx = pi(1,flag1)-po(1,flag1);
                dy = pi(2,flag1)-po(2,flag1);
                rad2(flag1) = dx.*dx+dy.*dy;
            end
            % Closest is first endpoint
            flag2 = v1dotv2 <= 0.0;
            if any(flag2)
                temp = bsxfun(@times,ones(2,sum(flag2)),[p1(1);p1(2)]);
                po(:,flag2) = temp;
                rad2(flag2) = inf;
            end
            % Closest is second endpoint
            flag3 = ~flag1 & ~flag2;
            if any(flag3)
                temp = bsxfun(@times,ones(2,sum(flag3)),[p2(1); p2(2)]);
                po(:,flag3) = temp;
                rad2(flag3) = inf;
            end
        end
     end    
end