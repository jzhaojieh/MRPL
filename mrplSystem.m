classdef mrplSystem < handle
    
    properties
            controllerObj,
            robot,
            trajectoryObj,
            trajFollower,
            rob
            enablePlot
            enableFeedback
            pid
            corXArr
            corYArr
            idealPoses
            encoderTimeStamp
            tstamp
            newenc
            tprev
            FaceOffset
            robFrontOffset
    end
    
    methods
        function obj = mrplSystem(sim, enablePlot, enableFeedback)
            if(sim)
                obj.robot = raspbot('sim');
            else
                obj.robot = raspbot('not sim');
            end
            obj.enablePlot = enablePlot;
            obj.enableFeedback = enableFeedback;
            obj.rob = robotModel();
            
            lRead = obj.robot.encoders.LatestMessage.Vector.X;
            rRead = obj.robot.encoders.LatestMessage.Vector.Y;
            obj.pid = controller(obj.rob, lRead, rRead);
            
            obj.corXArr = [];
            obj.corYArr = [];
            obj.idealPoses = pose(0,0,0);
            
            obj.encoderTimeStamp = 0;
            obj.robFrontOffset = .08;
            obj.FaceOffset = .019;
            obj.tstamp = 0;
            obj.newenc = [0,0];
            obj.tprev = 0;
        end
            
        function executeTrajectoryToPose(obj, x, y, th, sgn)
            goal = pose(x, y, th);
            currentPose = obj.idealPoses(end);
            convertedGoal = goal.matToPoseVec(currentPose.aToB() * goal.bToA());
            obj.executeTrajectoryRelativeToPose(convertedGoal(1), convertedGoal(2), convertedGoal(3), sgn);
        end
        
        function executeTrajectoryRelativeToPose(obj, x, y, th, sgn)
            obj.trajectoryObj = cubicSpiralTrajectory.planTrajectory(x, y, th, sgn);
            obj.trajectoryObj.planVelocities(0.2);
            obj.executeTrajectory();
        end
        

        function executeTrajectory(obj)
            obj.trajFollower = trajectoryFollower(obj.rob, obj.trajectoryObj);
            
            prevIdealPose = obj.idealPoses(end);
            
            obj.tstamp = obj.encoderTimeStamp;
            t = 0;
            timeFinal = 100;
            
            vl = 0;
            vr = 0;
            
            while(t < timeFinal)
                
                lRead = obj.newenc(1);
                rRead = obj.newenc(2);
                
                t = obj.encoderTimeStamp - obj.tstamp;
                
                if t == obj.tprev
                    obj.robot.sendVelocity(vl, vr);
                    pause(.005)
                    continue
                end
                
                corRelPose = obj.trajectoryObj.getPoseAtTime(t);
                worldPose = pose(0,0,prevIdealPose.th);
                
                %%%%%Convert relative coords to world coords%%%%%
                corPose = worldPose.bToA * [corRelPose(1); corRelPose(2); corRelPose(3)];

                obj.idealPoses = [obj.idealPoses, pose(corPose(1) + prevIdealPose.x, corPose(2) + prevIdealPose.y, corPose(3) + prevIdealPose.th)];
                
                [vlFF, vrFF] = obj.trajFollower.feedForwardVel(obj.trajFollower, t);
                [vlFB, vrFB] = obj.pid.giveError(obj.pid, lRead, rRead, t, obj.idealPoses(end));
                
                vl = obj.enableFeedback*vlFB + vlFF;
                vr = obj.enableFeedback*vrFB + vrFF;
                obj.robot.sendVelocity(vl, vr);
                
                corX = obj.idealPoses(end).x;
                corY = obj.idealPoses(end).y;
                
                obj.corXArr = [obj.corXArr, corX];
                obj.corYArr = [obj.corYArr, corY];
                
                if(obj.enablePlot)
                
                    subplot(1,2,1);
                    plot(obj.corXArr, obj.corYArr);
                    title("Correct Pose of Trajectory in meters");
                    xlabel("X distance (m)");
                    ylabel("Y distance (m)");
                    
                    subplot(1,2,2);
                    [actualXs, actualYs] = obj.pid.actualXY(obj.pid);
                    plot(actualXs, actualYs);
                    title("Current Pose of Trajectory in meters");
                    xlabel("X distance (m)");
                    ylabel("Y distance (m)");
                
                end
                
                obj.tprev = t;
                pause(.005);
                timeFinal = obj.trajectoryObj.getTrajectoryDuration();
            end
            obj.tstamp = 0;
            obj.robot.stop();
        end
        
        %objPoseVec is pose of object relative to sensor
        function robotGoal = acquisitionPose(obj, objPoseVec, moreOffset)
             curPose = obj.idealPoses(end);
             
             totalDist = obj.robFrontOffset + obj.FaceOffset + moreOffset;
             xCom = objPoseVec.x;
             yCom = objPoseVec.y;
             thCom = objPoseVec.th;
             xdelt = totalDist * cos(thCom);
             ydelt = totalDist * sin(thCom);
             x = xCom + xdelt;
             y = yCom + ydelt;
             th = thCom + tan(y/x);
             goalPose = pose(x,y,th);
             robotGoal = goalPose.matToPoseVec(curPose.aToB() * goalPose.bToA());

        end
        
        %=================From Lab 4=================================        
        function [uref, tf] = trapezoidalVelocityProfile( t , amax, vmax, dist, sgn)
        % Returns the velocity command of a trapezoidal profile of maximum
        % acceleration amax and maximum velocity vmax whose ramps are of
        % duration tf. Sgn is the sign of the desired velocities.
        % Returns 0 if t is negative. 
            tramp = vmax / amax;
            sf = dist;
            tf = (sf + vmax^2/amax)/vmax;

            if t < 0 || t >= tf
                uref = 0;
            elseif t < tramp
                uref = amax * t;
            elseif (tf - t) < tramp
                uref = amax * (tf - t);
            elseif tramp < t && t < (tf - tramp)
                uref = vmax;
            else
                uref = 0;
            end
            uref = uref * sgn;
        end
 
        function [rW, lW, tf] = trapezoidalAngVelocityProfile( t , amax, vmax, th)
        % Returns the velocity command of a trapezoidal profile of maximum
        % acceleration amax and maximum velocity vmax whose ramps are of
        % duration tf. Sgn is the sign of the desired velocities.
        % Returns 0 if t is negative. 
            tramp = vmax / amax;
            sf = mod (th, 360);
            if sf > 180
                sf = 180-sf;
            end
            sgn = 1
            if sf < 0 
                sgn = -1
            end
            tf = (sf + vmax^2/amax)/vmax;

            if t < 0 || t >= tf
                uref = 0;
            elseif t < tramp
                uref = amax * t;
            elseif (tf - t) < tramp
                uref = amax * (tf - t);
            elseif tramp < t && t < (tf - tramp)
                uref = vmax;
            else
                uref = 0;
            end
            rW = uref * sgn;
            lW = uref;
        end
      
%=====================================================
               
        function moveRelDist(obj, dist, doControlPlotting)
            %move forward or backward a specified distance and stop
            sgn = 1;
            if dist < 0
                sgn = 1;
            end
            %make sure the velocity is such that the distance will take at
            %least a second    
            [vel, tf] = trapezoidalVelocityProfile(obj.tstamp, obj.aMax, obj.vMax, dist, sgn);
            if tf >= 1
            %move distance forward or backward
                obj.robot.sendVelocity(vel, vel);
            else
                disp("moveRelDist had insufficient velocity, takes less than a second");
            end
        end

        function turnRelAngle(obj, angle, doControlPlotting)
            % make sure the velocity is such that the distance will take at
            % least a second
            [rw, lw, tf] = trapezoidalAngVelocityProfile(obj.tstamp, obj.aMax, objvMax, angle);
            if tf >= 1
                obj.robot.sendVelocity(rw, lw);
            else
                disp("turnRelAngle had insufficient velocity, takes less than a second");
            end
    end
end