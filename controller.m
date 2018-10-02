%% Controller
classdef controller
    properties
        robTraj
        pArr
    end
    
    methods
        function obj = controller(robTraj)
            obj.robTraj = robTraj;
        end
        
        function [V, w, [rError, theta]] = pid(obj, time, curPose)
            tdelay = 0.2;
            dPose = obj.robTraj.getPoseForTime(time - tdelay);
            theta = atan2(sin(dPose(3) - curPose(3)), cos(dPose(3) - curPose(3)));
            perror = pose(dPose(1) - curPose(1), dPose(2) - curPose(2), theta);
            rError = [dPose(1) - curPose(1), dPose(2) - curPose(2)] * perror.bToARot();
            
            
            V = .02*rError(1);
            if V < .0001
                rError(2) = 0;
            end
            w = .01*rError(2) + .02*theta;
         
        end
    end
end