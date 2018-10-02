
%% Robot Trajectory

classdef robotTrajectory < handle
    properties
        refcontrol
        numsamples
        tArr
        dArr
        vArr
        pArr
        wArr
    end
    methods 
        function obj = robotTrajectory(refcontrol, numsamples)
            obj.refcontrol = refcontrol;
            obj.numsamples = numsamples;
        end
        
        function generateSamples(obj)
            V = 0;
            w = 0;
            dist = 0;
            angle = 0;
%             angle = 0;
            x = 0;
            y = 0;
            p = [x, y, angle];
            obj.tArr = [];
            obj.dArr = [];
            obj.vArr = [];
            obj.wArr = [];
            obj.pArr = [];
            dt = obj.refcontrol.totalTime / obj.numsamples;
            for i = 1:(obj.numsamples)
                ti = (i-1)*dt;
                obj.tArr = [obj.tArr, ti]; %time update
                [V, w] = obj.refcontrol.computeControl(ti);
                obj.vArr = [obj.vArr V]; %velocity update
                obj.wArr = [obj.wArr w];
                dist = dist + (V * dt);
                obj.dArr = [obj.dArr, dist]; %distance update
                angle = angle + w*dt;
                x = V*dt*sin(angle);
                y = V*dt*cos(angle);
                p = [x; y; angle];
                obj.pArr = [obj.pArr, p]; %position update
            end
        end
        function vel = getVelForTime(obj, t)
            vel = interp1(obj.tArr, transpose(obj.vArr), t);
        end
        
        function w = getWForTime(obj, t)
            w = interp1(obj.tArr, transpose(obj.wArr), t);
        end
        
        function dist = getDistForTime(obj, t)
            dist = interp1(obj.tArr,  transpose(obj.dArr), t);
        end
        
        function pose = getPoseForTime(obj, t)
            pose = interp1(obj.tArr, transpose(obj.pArr), t);
        end
    end
end