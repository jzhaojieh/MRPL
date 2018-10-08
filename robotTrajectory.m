classdef robotTrajectory < handle
    properties(Constant)
        initialPose = pose(0,0,0)
    end
    properties(Access = public)
        numSamples;
        refCon;
        trajTotalTime;
        timeArr = [];
        distArr = [];
        velArr = [];
        wArr = [];
        poseArr = [];
    end
    properties(Access = private)
    end
    methods(Static = true)
        function obj = robotTrajectory(numberSamples, rC)
            object.numSamples = numberSamples;
            object.refCon = rC;
            object.trajTotalTime = object.refCon.getTrajectoryDuration();
            object.timeArr = [object.timeArr, 0];
            object.distArr = [object.distArr, 0];
            object.velArr = [object.velArr, 0];
            object.wArr = [object.wArr, 0];
            object.poseArr = [object.poseArr, object.initialPose];
        end
        function generateSamples(obj)
            dt = object.trajTotalTime / object.numSamples;
            for i = 1:(object.numSamples - 1)
                ti = (i-1)*dt;
                [V, w] = object.refCon.computeControl(ti);
                
                %------THIS PART MIGHT BE WRONG-------
                angle = wArr(i) + (w*dt);
                x = V*dt*sin(angle);
                y = V*dt*cos(angle);
                %I'M NOT SURE IF THIS IS p(i+1) AS REQUIRED???
                p = pose(x, y, angle);
                %--------------------------------------
                
                %updates
                object.timeArr = [object.timeArr, ti];
                object.velArr = [object.velArr, V];
                object.wArr = [object.wArr, w];
                object.distArr(i+1) = object.distArr(i) + (V*dt);
                object.poseArr(i+1) = p;
            end
        end
        function linVel = getVelocity(obj, t)
            linVel = interpl(object.timeArr, transpose(obj.velArr), t);
        end
        function angVel = getW(obj, t)
            angVel = interpl(object.timeArr, transpose(obj.wArr), t);
        end
        function curDist = getDist(obj, t)
            curDist = interpl(object.timeArr, transpose(obj.distArr), t);
        end
        function curPose = getPoseAtTime(obj, t)
            curPose = interpl(object.timeArr, transpose(obj.poseArr), t);
        end
    end
end