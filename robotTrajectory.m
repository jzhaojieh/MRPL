classdef robotTrajectory < handle
    properties(Constant)
    end
    properties(Access = public)
        numSamples;
        refCon;
        trajTotalTime;
        timeArr;
        distArr;
        velArr;
        wArr;
        poseXArr;
        poseYArr;
        poseThArr;
    end
    properties(Access = private)
    end
    methods(Static = true)
        function obj = robotTrajectory(numberSamples, rC)
            obj.numSamples = numberSamples;
            obj.refCon = rC;
            obj.trajTotalTime = obj.refCon.getTrajectoryDuration(obj.refCon);
            obj.timeArr = [];
            obj.distArr = [];
            obj.velArr = [];
            obj.wArr = [];
            obj.poseXArr = [];
            obj.poseYArr =[];
            obj.poseThArr = [];
          
            obj.timeArr = [obj.timeArr, 0];
            obj.distArr = [obj.distArr, 0];
            obj.velArr = [obj.velArr, 0];
            obj.wArr = [obj.wArr, 0];
            obj.poseXArr = [obj.poseXArr, 0];
            obj.poseYArr =[obj.poseYArr, 0];
            obj.poseThArr = [obj.poseThArr, 0];
        end
        function generateSamples(obj)
            dt = obj.trajTotalTime / obj.numSamples;
            for i = 1:(obj.numSamples - 1)
                timeInterval = (i-1)*dt;
                [V, w] = obj.refCon.computeControl(obj.refCon, timeInterval);
                
                %------THIS PART MIGHT BE WRONG-------
                angle = obj.wArr(i) + (w*dt);
                x = V*dt*sin(angle);
                y = V*dt*cos(angle);
                %--------------------------------------
                
                %updates
                obj.timeArr = [obj.timeArr, timeInterval];
                obj.velArr = [obj.velArr, V];
                obj.wArr = [obj.wArr, w];
                obj.distArr(i+1) = obj.distArr(i) + (V*dt);
                obj.poseXArr(i+1) = x;
                obj.poseYArr(i+1) = y;
                obj.poseThArr(i+1) = angle;
            end
        end
        function linVel = getVelocity(obj, t)
            linVel = interp1q(obj.timeArr, transpose(obj.velArr), t);
        end
        function angVel = getW(obj, t)
            angVel = interp1q(obj.timeArr, transpose(obj.wArr), t);
        end
        function curDist = getDist(obj, t)
            curDist = interp1q(obj.timeArr, transpose(obj.distArr), t);
        end
        function curPose = getPoseAtTime(obj, t)
            curX = interp1q(obj.timeArr, transpose(obj.poseXArr), t);
            curY = interp1q(obj.timeArr, transpose(obj.poseYArr), t);
            curTh = interp1q(obj.timeArr, transpose(obj.poseThArr), t);
            curPose = pose(curX, curY, curTh);
        end
    end
end