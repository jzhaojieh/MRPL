classdef mrplSystem
    
    properties
            controllerObj,
            robot,
            trajectoryObj
    end
    
    methods
        function obj = mrplSystem()
%             obj.robot = raspbot('Raspbot-07');
            obj.robot = raspbot('sim');
            pause(.1);
            xf1 = 0.3048; yf1 = 0.3048; thf1 = 0.0;
            obj.executeTrajectoryToRelativePose(xf1, yf1, thf1, 1, 1);
            pause(.1);
        end
            
        function executeTrajectoryToRelativePose(obj, x, y, th, sgn, iteration)
            obj.trajectoryObj = cubicSpiralTrajectory.planTrajectory(x, y, th, sgn);
            obj.trajectoryObj.planVelocities(0.2);
            obj.executeTrajectory();
        end
        

        function executeTrajectory(obj)
            
        end
    end
        
end