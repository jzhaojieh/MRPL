classdef controller < handle
    properties(Constant)
    end
    properties(Access = private)
    end
    properties(Access = public)
        rob;
        roboTraj;
        lEnc;
        rEnc;
        thArr;
        timeArr;
        actualPoses;
        linError;
        angError;
    end
    methods(Static = true)
        function obj = controller(robotModel, robotTrajectory, lRead, rRead)
            obj.rob = robotModel;
            obj.roboTraj = robotTrajectory;
            initialPose = pose(0,0,0);
            obj.lEnc = [];
            obj.rEnc = [];
            obj.thArr = [];
            obj.timeArr = [];
            obj.actualPoses = [];
            obj.linError = [];
            obj.angError = [];
            obj.lEnc = [obj.lEnc, lRead];
            obj.rEnc = [obj.rEnc, rRead];
            obj.thArr = [obj.thArr, 0];
            obj.timeArr = [obj.timeArr, 0];
            obj.actualPoses = [obj.actualPoses, initialPose]; %robotCoord poses that were measured encoders
            obj.linError = [obj.linError, 0];
            obj.angError = [obj.angError, 0];
        end
        function [uv, uw] = giveError(obj, lRead, rRead, tcur)
            %curPose is where robot is now, derived from lRead, rRead
            pPose = obj.actualPoses(end); %previous pose 
            plEnc = obj.lEnc(end); %previous lEnc
            prEnc = obj.rEnc(end); %previous rEnc
            tprev = obj.timeArr(end);
            
            vl = (lRead - plEnc) / (tcur - tprev);
            vr = (rRead - prEnc) / (tcur - tprev);
            [V, w] = obj.rob.vlvrToVw(vl, vr);
            
            dTheta = w*(tcur - tprev);
            curTh = obj.thArr(end) + dTheta;
            displacement = V*(tcur - tprev);
            dx = displacement*cos(curTh);
            dy = displacement*sin(curTh);
            
            prevX = pPose.x;
            prevY = pPose.y;
            prevTh = pPose.th;
            curPose = pose(prevX + dx, prevY + dy, curTh);
            
            %----Convert World Coord to RobotCoord-------
            correctPos = obj.roboTraj.getPoseAtTime(tcur);
            corTh = correctPos(3);
            
            convertMatrix = zeros(2, 2);
            convertMatrix(1,1) = cos(corTh); 
            convertMatrix(1,2) = -sin(corTh);
            convertMatrix(2,1) = sin(corTh); 
            convertMatrix(2,2) = cos(corTh);
            
            wrrp = zeros(2,1);
            wrrp(1,1) = curPose.x; wrrp(2,1) = curPose.y;
            
            rrp = convertMatrix*wrrp;
            
            errorX = rrp(1,1);
            errorY = rrp(2,1);
            errorTh = atan2(sin(corTh-curPose.th), cos(corTh-curPose.th));

            %----Tau stuff--------
            tau = 200;
            kx = 1/tau;
            ky = 2/(tau^2*abs(V));
            kth = 1/tau;
            %---------------------
            
            if V < .001
                ky = 0;
            end
            uv = kx*errorX;
            uw = ky*errorY + kth*errorTh;
            
            obj.lEnc = [obj.lEnc, lRead];
            obj.rEnc = [obj.rEnc, rRead];
            obj.timeArr = [obj.timeArr, tcur];
            obj.actualPoses = [obj.actualPoses, curPose];
            obj.linError = [obj.linError, sqrt(errorX^2 + errorY^2)];
            obj.angError = [obj.angError, errorTh];
            obj.thArr = [obj.thArr, curPose.th];
            %-----------------------------
        end
    end
end