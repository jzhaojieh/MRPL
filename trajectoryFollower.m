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
            obj.rob = robModel;
            obj.W2 = obj.rob.W2;
            obj.roboTraj = robotTraject;
        end
        function [vl, vr] = feedForwardVel(obj, t)
            V = obj.roboTraj.getVAtTime(t);
            w = obj.roboTraj.getwAtTime(t);
            pose = obj.roboTraj.getPoseAtTime(t);
            
            %updating arrays
            obj.timeArr = [obj.timeArr, t];
            obj.velArr = [obj.velArr, V];
            obj.wArr = [obj.wArr, w];
            obj.poseArr = [obj.poseArr, pose];
            
            %returning velocityLeft and velocityRight
            vl = V - obj.W2*w;
            vr = V + obj.W2*w;
        end
    end
end