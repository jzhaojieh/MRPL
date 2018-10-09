classdef controller < handle
    properties(Constant)
    end
    properties(Access = private)
    end
    properties(Access = public)
        rob;
        robTraj;
        lEnc = [];
        rEnc = [];
        timeArr = [];
        actualPoses = [];
        linError = [];
        angError = [];
    end
    methods(Static = true)
        function obj = controller(robotModel, robotTrajectory, lRead, rRead)
            object.rob = robotModel;
            object.robTraj = robotTrajectory;
            initialPose = pose(0,0,0);
            lEnc = [lEnc, lRead];
            rEnc = [rEnc, rRead];
            timeArr = [timeArr, 0];
            actualPoses = [actualPoses, initialPose]; %robotCoord poses that were measured encoders
            linError = [linError, 0];
            angError = [angError, 0];
        end
        function [uv, uw] = giveError(lRead, rRead, tcur)
            %curPose is where robot is now, derived from lRead, rRead
            pPose = actualPoses(end); %previous pose 
            plEnc = lEnc(end); %previous lEnc
            prEnc = rEnc(end); %previous rEnc
            tprev = timeArr(end);
            
            vl = (lEnc - plEnc) / (tcur - tprev);
            vr = (rEnc - prEnc) / (tcur - tprev);
            [V, w] = object.rob.vlvrToVw(vl, vr);
            
            dTheta = w*(tcur - tprev);
            displacement = V*(tcur - tprev);
            dx = displacement*sin(dTheta);
            dy = displacement*cos(dTheta);
            
            prevX = pPose.x;
            prevY = pPose.y;
            prevTh = pPose.th;
            
            curPose = pose(prevX + dx, prevY + dy, prevTh + th);
            
            %----Convert World Coord to RobotCoord-------
            correctPos = object.roboTraj.getPoseAtTime(tcur);
            corX = correctPos.x;
            corY = correctPos.y;
            corTh = correctPos.th;
%             Twr = zeros(3,3);
%             Twr(1,1) = cos(theta); Twr(1,2) = -sin(theta); Twr(1,3) = curPose.x;
%             Twr(2,1) = sin(theta); Twr(2,2) = cos(theta); Twr(2,3) = curPose.y;
%             Twr(3,1) = 0; Twr(3,2) = 0; Twr(3,3) = 1;
%             TwrInv = inv(Twr);
%             
%             rwp = zeros(3,1);
%             rwp(1,1) = corX; rwp(2,1) = corY; rwp(3,1) = corTh;
            
            convertMatrix = zeros(2, 2);
            convertMatrix(1,1) = cos(curPose.th); convertMatrix(1,2) = -sin(curPose.th);
            convertMatrix(2,1) = sin(curPose.th); convertMatrix(2,2) = cos(curPose.th);
            
            wrrp = zeros(2,1);
            wrrp(1,1) = curPose.x; wrrp(2,1) = curPose.y;
            
            rrp = convertMatrix*wrrp;
            
            errorX = rrp(1,1);
            errorY = rrp(2,1);
            errorTh = atan2(sin(curPose.th-corTh), cos(curPose.th-corTh));

            %----Tau stuff--------
            tau = 0.25;
            kx = 1/tau;
            ky = 2/(tau^2*abs(V));
            kth = 1/tau;
            %---------------------
            
            if V < .001
                ky = 0;
            end
            uv = kx*errorX;
            uw = ky*errorY + kth*errorTh;
            
            lEnc = [lEnc, lRead];
            rEnc = [rEnc, rRead];
            timeArr = [timeArr, tcur];
            actualPoses = [actualPoses, curPose];
            linError = [linError, sqrt(errorX^2 + errorY^2)];
            angError = [angError, errorTh];
            %-----------------------------
        end
    end
end