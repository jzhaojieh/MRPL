classdef mrplSystem < handle
    
    properties
            controllerObj,
            robot,
            trajectoryObj,
            trajFollower,
            rob
    end
    
    methods
        function obj = mrplSystem()
            obj.robot = raspbot('raspberrybot #7 MRPL af');
            pause(.1);
            xf1 = 0.3048; yf1 = 0.3048; thf1 = 0.0;
            xf2 = -0.6096; yf2 = -0.6096; thf2 = -pi()/2.0;
            xf3 = -0.3048; yf3 = 0.3048; thf3 = pi()/2.0;
            obj.executeTrajectoryToRelativePose(xf1, yf1, thf1, 1, 1);
            pause(.1);
            obj.executeTrajectoryToRelativePose(xf2, yf2, thf2, 1, 3);
            pause(.1);
            obj.executeTrajectoryToRelativePose(xf3, yf3, thf3, 1, 5);
            pause(.1);
            obj.robot.shutdown();
        end
            
        function executeTrajectoryToRelativePose(obj, x, y, th, sgn, iteration)
            obj.trajectoryObj = cubicSpiralTrajectory.planTrajectory(x, y, th, sgn);
            obj.trajectoryObj.planVelocities(0.2);
            obj.executeTrajectory(iteration);
        end
        

        function executeTrajectory(obj, i)
            obj.rob = robotModel();
            obj.trajFollower = trajectoryFollower(obj.rob, obj.trajectoryObj);
            
            lRead = obj.robot.encoders.LatestMessage.Vector.X;
            rRead = obj.robot.encoders.LatestMessage.Vector.Y;
            pid = controller(obj.rob, obj.trajectoryObj, lRead, rRead);
            start = tic;
            t = toc(start);

            %=========TOGGLE PID========================
            enable = 0;
            timeFinal = 100;
            %============================================
            actualXArr = [];
            actualYArr = [];
            corXArr = [];
            corYArr = [];
            while(t < timeFinal)
                lRead = obj.robot.encoders.LatestMessage.Vector.X;
                rRead = obj.robot.encoders.LatestMessage.Vector.Y;
                t = toc(start);
                [vlFF, vrFF] = obj.trajFollower.feedForwardVel(obj.trajFollower, t);
                [vlFB, vrFB] = pid.giveError(pid, lRead, rRead, t);
                vl = enable*vlFB + vlFF;
                vr = enable*vrFB + vrFF;
                obj.robot.sendVelocity(vl, vr);
                
                
                subplot(3,2,i);
                corPose = obj.trajectoryObj.getPoseAtTime(t);
                corX = corPose(1);
                corY = corPose(2);
                corXArr = [corXArr, corX];
                corYArr = [corYArr, corY];
                
                plot(corXArr, corYArr);
                title("Correct Pose of Trajectory in meters");
                xlabel("X distance (m)");
                ylabel("Y distance (m)");
                
                subplot(3,2,i+1);
                actualPose = pid.actualPoses(end);
                actualX = actualPose.x;
                actualY = actualPose.y;
                actualXArr = [actualXArr, actualX];
                actualYArr = [actualYArr, actualY];
                
                plot(actualXArr, actualYArr);
                title("Current Pose of Trajectory in meters");
                xlabel("X distance (m)");
                ylabel("Y distance (m)");
                
                pause(.05);
                timeFinal = obj.trajectoryObj.getTrajectoryDuration();
            end
            obj.robot.stop();
            
        end
    end
        
end