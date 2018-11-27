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
        actualXs;
        actualYs;
        linError;
        angError;
    end
    methods(Static = true)
        function obj = controller(robotModel, lRead, rRead)
            obj.rob = robotModel;
            initialPose = pose(0,0,0);
            obj.lEnc = [];
            obj.rEnc = [];
            obj.thArr = [];
            obj.timeArr = [];
            obj.actualPoses = [];
            obj.actualXs = [obj.actualXs, 0];
            obj.actualYs = [obj.actualYs, 0];
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
        
        
        function [uv, uw] = giveError(obj, lRead, rRead, tcur, correctPos)
            %curPose is where robot is now, derived from lRead, rRead
            plEnc = obj.lEnc(end); %previous lEnc
            prEnc = obj.rEnc(end); %previous rEnc
            tprev = obj.timeArr(end);
            
            dt = tcur - tprev;
            
            if dt == 0
                uv = 0;
                uw = 0;
                return
            end
            
            vl = (lRead - plEnc) / (dt);
            vr = (rRead - prEnc) / (dt);
            [V, w] = obj.rob.vlvrToVw(vl, vr);
            
            prevX = obj.actualXs(end);
            prevY = obj.actualYs(end);
            prevTh = obj.thArr(end);
            
            dTheta = w*(dt);
            curTh = prevTh + dTheta;
            
            %%%%Normalizing the angle%%%%
            if abs(curTh) > pi
                curTh = sign(curTh)*(-1)*(2*pi-abs(curTh));
            end
            
            displacement = V*(dt);
            dx = displacement*cos(curTh-(dTheta/2));
            dy = displacement*sin(curTh-(dTheta/2));
            
            curPose = pose(prevX + dx, prevY + dy, curTh);
            
            errorX = correctPos.x - curPose.x;
            errorY = correctPos.y - curPose.y;
            errorTh = atan2(sin(correctPos.th-curPose.th), cos(correctPos.th-curPose.th));
            
            errorConverted = curPose.bToARot() * [errorX; errorY];

            %----Tau stuff--------
            tau = 20;
            kx = 1/tau;
            if V < .001
                ky = 0;
            else
                ky = 2/(tau^2*abs(V));
            end
            kth = 1/tau;
            %---------------------
            
            uv = kx*errorConverted(1);
            uw = ky*errorConverted(2) + kth*errorTh;
            
            obj.lEnc = [obj.lEnc, lRead];
            obj.rEnc = [obj.rEnc, rRead];
            obj.timeArr = [obj.timeArr, tcur];
            obj.actualPoses = [obj.actualPoses, curPose];
            obj.actualXs = [obj.actualXs, curPose.x];
            obj.actualYs = [obj.actualYs, curPose.y];
            obj.linError = [obj.linError, sqrt(errorX^2 + errorY^2)];
            obj.angError = [obj.angError, errorTh];
            obj.thArr = [obj.thArr, curPose.th];
            %-----------------------------
        end
        
        function [a, b] = actualXY(obj)
            a = obj.actualXs;
            b = obj.actualYs;
        end
    end
end