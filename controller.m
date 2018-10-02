%% Controller
classdef controller
    properties
        robTraj
    end
    
    methods
        function obj = controller(robTraj)
            obj.robTraj = robTraj;
        end
        
        function [V w] = pid(time, curPose)
            tdelay = 0.2;
            dPose = robotTrajectory.getPoseForTime(obj.robTraj, time - tdelay);
            perror = pose(dPose(1) - curPose(1), dPose(2) - curPose(2), dPose(3) - curPose(3));
            rError = perror * pose.bToARot(perror);
            perror(3) = atan2(sin(perror(3)), cos(perror(3)));
            
            V = .2*perror(1);
            if V < .001
                perror(2) = 0;
            end
            w = .1*perror(2) + .2*perror(3);
        end
    end
end