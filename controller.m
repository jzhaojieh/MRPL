%% Controller
classdef controller
    properties
        robTraj
    end
    
    methods
        function obj = controller(robTraj)
            obj.robTraj = robTraj;
        end
        
        function [V, w] = pid(obj, time, curPose)
            tdelay = 0.2;
            dPose = obj.robTraj.getPoseForTime(time - tdelay);
            perror = pose(dPose(1) - curPose(1), dPose(2) - curPose(2), dPose(3) - curPose(3));
            rError = [dPose(1) - curPose(1), dPose(2) - curPose(2)] * perror.bToARot();
            theta = atan2(sin(perror.th), cos(perror.th));
            
            V = .2*rError(1);
            if V < .001
                rError(2) = 0;
            end
            w = .1*rError(2) + .2*theta;
        end
    end
end