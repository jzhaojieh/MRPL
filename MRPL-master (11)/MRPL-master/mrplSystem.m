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
            vmax
            amax
            eold
            eint
    end
    
    methods
        function obj = mrplSystem(sim, enablePlot, enableFeedback)
            if(sim)
                obj.robot = raspbot('sim');
                
                ob1 = lineObject;
                ob1.lines = [0 -.0635; 0 0.0635];
                ob1.color = [1 0 0];
                ob1.pose = [.45 0 0];
                
                ob2 = lineObject;
                ob2.lines = [0 -.0635; 0 0.0635];
                ob2.color = [1 0 0];
                ob2.pose = [-.45 0 0];
                
                ob3 = lineObject;
                ob3.lines = [-.5, -.5; -.5, .5; .5, .5; .5, -.5; -.5, -.5];
                ob3.color = [1 0 0];
                ob3.pose = [0 0 0];
                
                obj.robot.genMap([ob1, ob2, ob3]);
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
            
            obj.vmax = .05;
            obj.amax = .1;
            
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
            convertedGoal = pose.matToPoseVec(currentPose.aToB() * goal.bToA());
            obj.executeTrajectoryRelativeToPose(convertedGoal(1), convertedGoal(2), convertedGoal(3), sgn);
        end
        
        function executeTrajectoryRelativeToPose(obj, x, y, th, sgn)
            obj.trajectoryObj = cubicSpiralTrajectory.planTrajectory(x, y, th, sgn);
            obj.trajectoryObj.planVelocities(0.2);
            obj.executeTrajectory(sgn);
        end
        

        function executeTrajectory(obj, sgn)
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
                idealTh = (corPose(3) + prevIdealPose.th);
                if abs(idealTh) > pi
                    idealTh = sign(idealTh)*(-1)*(2*pi-abs(idealTh));
                end
                obj.idealPoses = [obj.idealPoses, pose(corPose(1) + prevIdealPose.x, corPose(2) + prevIdealPose.y, idealTh)];
                
                [vlFF, vrFF] = obj.trajFollower.feedForwardVel(obj.trajFollower, t);
                [vlFB, vrFB] = obj.pid.giveError(obj.pid, lRead, rRead, t, obj.idealPoses(end));
                
                vl = obj.enableFeedback*vlFB + vlFF;
                vr = obj.enableFeedback*vrFB + vrFF;
                obj.robot.sendVelocity(vl*sgn, vr*sgn);
                
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
      
%=====================================================
               
        function moveRelDist(obj, dist)
            %move forward or backward a specified distance and stop
            obj.eold = 0;
            obj.eint = 0;
            %make sure the velocity is such that the distance will take at
            %least a second  
            sf = dist;
            tf = (abs(sf) + obj.vmax^2/obj.amax)/obj.vmax;
            tstart = obj.encoderTimeStamp;
            lstart = obj.newenc(1);
            rstart = obj.newenc(2);
            ddelay = 0;
            prevt = 0;
            tdelay = .02;
            t = obj.encoderTimeStamp - tstart;
            prevIdealPose = obj.idealPoses(end);
            obj.idealPoses = [obj.idealPoses, pose(prevIdealPose.x+dist*cos(prevIdealPose.th), prevIdealPose.y+dist*sin(prevIdealPose.th), prevIdealPose.th)];
            curDist = 0;
            
            while (t < (tf + 1) && abs(dist - (curDist)) > 0.0001)
                lRead = obj.newenc(1);
                rRead = obj.newenc(2);
                t = obj.encoderTimeStamp - tstart;
                dt = t-prevt;
                prevt = t;
                [~, ~] = obj.pid.giveError(obj.pid, lRead, rRead, t, obj.idealPoses(end));
                dl = lRead-lstart;
                dr = rRead-rstart;
                curDist = (dl + dr)/2;
                uref = obj.trapezoidalVelocityProfile(t , dist, sign(dist));
                ddelay = ddelay + obj.trapezoidalVelocityProfile(t - tdelay, dist, sign(dist))*dt;
                upid = obj.getpid(curDist, dt, ddelay);
                
                u = upid + uref;

                obj.robot.sendVelocity(u, u);
                
                pause(.05);
                lRead = obj.newenc(1);
                rRead = obj.newenc(2);
                t = obj.encoderTimeStamp - tstart;
                [~, ~] = obj.pid.giveError(obj.pid, lRead, rRead, t, obj.idealPoses(end));
            end
            obj.robot.stop();
        end

        function turnRelAngle(obj, angle)
            % make sure the velocity is such that the distance will take at
            % least a second
            %make sure the velocity is such that the distance will take at
            %least a second  
            finalAngle = mod (angle, 360);
            if finalAngle > 180
                finalAngle = 180-finalAngle;
            end
            sgn = 1;
            if finalAngle < 0 
                sgn = -1;
            end
            sf = finalAngle * (pi/180) * obj.rob.W2;
            tf = (abs(sf) + obj.vmax^2/obj.amax)/obj.vmax;
            
            obj.tstamp = obj.encoderTimeStamp;
            
            prevIdealPose = obj.idealPoses(end);
            
            idealAngle = prevIdealPose.th + finalAngle*(pi/180);
            idealAngle = mod(idealAngle, 2*pi);
            if idealAngle > pi
                idealAngle = idealAngle-2*pi;
            end
            
            obj.idealPoses = [obj.idealPoses, pose(prevIdealPose.x, prevIdealPose.y, idealAngle)];

            prevActualPose = obj.pid.actualPoses(end);
            
            actualAngle = prevActualPose.th + finalAngle*(pi/180);
            actualAngle = mod(actualAngle, 2*pi);
            if actualAngle > pi
                actualAngle = actualAngle-2*pi;
            end
            
            obj.pid.actualPoses = [obj.pid.actualPoses, pose(prevActualPose.x, prevActualPose.y, actualAngle)];
            
            obj.eold = 0;
            obj.eint = 0;

            tstart = obj.encoderTimeStamp;
            lstart = obj.newenc(1);
            rstart = obj.newenc(2);
            ddelay = 0;
            prevt = 0;
            tdelay = .02;
            t = obj.encoderTimeStamp - tstart;
            curDist = 0;
            
            while (t < (tf+1) && abs(sf - (curDist)) > 0.0001)
                lRead = obj.newenc(1);
                rRead = obj.newenc(2);
                t = obj.encoderTimeStamp - tstart;
                dt = t-prevt;
                prevt = t;
                %[~, ~] = obj.pid.giveError(obj.pid, lRead, rRead, t, obj.idealPoses(end));
                dl = lRead-lstart;
                dr = rRead-rstart;
                curDist = (abs(dl) + abs(dr))/2;
                uref = obj.trapezoidalVelocityProfile(t , sf, sgn);
                ddelay = ddelay + obj.trapezoidalVelocityProfile(t - tdelay, sf, sgn)*dt;
                upid = obj.getpid(curDist, dt, ddelay);
                
                u = upid + uref;

                obj.robot.sendVelocity(u, -u);
                
                pause(.05);
            end
            obj.robot.stop();
        end
        
        function upid = getpid(obj, d, dt, ddelay)
            kp = 1;
            kd = .01;
            ki = .005;

            enow = ddelay-d;   
            if dt ~= 0
                edir = abs((enow - obj.eold)/dt);
            else
                edir = 0;
            end
            obj.eint = obj.eint + (enow * dt);
            if abs(obj.eint) > 0.1
                if obj.eint > 0
                    obj.eint = 0.1;
                else
                    obj.eint = -1 * 0.1;
                end
            end
            upid = (enow * kp) + (edir * kd) + (obj.eint * ki);
            if abs(upid) > 0.3
                if upid > 0
                    upid = 0.3;
                else
                    upid = -1 * 0.3;
                end
            end
            obj.eold = enow;
        end
        
        function uref = trapezoidalVelocityProfile(obj, t, dist, sgn)
            % Returns the velocity command of a trapezoidal profile of maximum
            % acceleration amax and maximum velocity vmax whose ramps are of
            % duration tf. Sgn is the sign of the desired velocities.
            % Returns 0 if t is negative. 
            tramp = obj.vmax / obj.amax;
            sf = dist;
            tf = (abs(sf) + obj.vmax^2/obj.amax)/obj.vmax;
            
            if t < 0 || t >= tf
                uref = 0;
            elseif t < tramp
                uref = obj.amax * t;
            elseif (tf - t) < tramp
                uref = obj.amax * (tf - t);
            elseif tramp < t && t < (tf - tramp)
                uref = obj.vmax;
            else
                uref = 0;
            end
            uref = uref * sgn;
        end
    end
end