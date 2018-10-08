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
        function obj = trajectoryFollower(robModel, robotTraject)
            object.rob = robModel;
            object.W2 = object.rob.W2;
            object.roboTraj = robotTraject;
        end
        function [vl, vr] = feedForwardVel(obj, t)
            V = object.roboTraj.getVelocity(t)
            w = object.roboTraj.getW(t);
            pose = object.roboTraj.getPoseAtTime(t);
            
            %updating arrays
            timeArr = [timeArr, t];
            velArr = [velArr, V];
            wArr = [wArr, w];
            poseArr = [poseArr, pose];
            
            %returning velocityLeft and velocityRight
            vl = V - object.W2*w;
            vr = V + object.W2*w;
        end
    end
end