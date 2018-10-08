classdef trajectoryFollower < handle
    properties(Constant)
    end
    properties(Access = public)
        rob;
        W2;
        numSamples;
        refCon; 
        roboTraj;
        timeArr = [];
        velArr = [];
        wArr = [];
        poseArr = [];
    end
    properties(Access = private)
    end
    methods(Static = true)
        function obj = trajectoryFollower(robModel, numSamps, referenceControl)
            object.rob = robModel;
            object.W2 = object.rob.W2
            object.numSamples = numSamps;
            object.refCon = referenceControl;
            object.roboTraj = robotTrajectory(object.numSamples, object.refCon);
        end
        function [vl, vr] = feedForwardVel(obj, t)
            V = object.roboTraj.getVelocity(t)
            w = object.roboTraj.getW(t);
            pose = object.roboTraj.getPoseAtTime(t);
            vl = V - object.W2*w;
            vr = V + object.W2*w;
            timeArr = [timeArr, t];
            velArr = [velArr, V];
            wArr = [wArr, w];
            poseArr = [poseArr, pose];
        end
    end
end