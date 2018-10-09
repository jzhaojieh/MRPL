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
        poseArr;
    end
    properties(Access = private)
    end
    methods(Static = true)
        function obj = robotTrajectory(numberSamples, rC)
            object.numSamples = numberSamples;
            object.refCon = rC;
            object.trajTotalTime = object.refCon.getTrajectoryDuration(object.refCon);
            object.timeArr = [];
            object.distArr = [];
            object.velArr = [];
            object.wArr = [];
            object.poseArr = [];
            initialPose = pose(0,0,0);
          
            object.timeArr = [object.timeArr, 0];
            object.distArr = [object.distArr, 0];
            object.velArr = [object.velArr, 0];
            object.wArr = [object.wArr, 0];
            object.poseArr = [object.poseArr, initialPose];
        end
        function generateSamples(object)
            dt = object.trajTotalTime / object.numSamples;
            for i = 1:(object.numSamples - 1)
                timeInterval = (i-1)*dt;
                [V, w] = object.refCon.computeControl(timeInterval);
                
                %------THIS PART MIGHT BE WRONG-------
                angle = wArr(i) + (w*dt);
                x = V*dt*sin(angle);
                y = V*dt*cos(angle);
                %I'M NOT SURE IF THIS IS p(i+1) AS REQUIRED???
                p = pose(x, y, angle);
                %--------------------------------------
                
                %updates
                object.timeArr = [object.timeArr, timeInterval];
                object.velArr = [object.velArr, V];
                object.wArr = [object.wArr, w];
                object.distArr(i+1) = object.distArr(i) + (V*dt);
                object.poseArr(i+1) = p;
            end
        end
        function linVel = getVelocity(object, t)
            linVel = interpl(object.timeArr, transpose(object.velArr), t);
        end
        function angVel = getW(object, t)
            angVel = interpl(object.timeArr, transpose(object.wArr), t);
        end
        function curDist = getDist(object, t)
            curDist = interpl(object.timeArr, transpose(object.distArr), t);
        end
        function curPose = getPoseAtTime(object, t)
            curPose = interpl(object.timeArr, transpose(object.poseArr), t);
        end
    end
end