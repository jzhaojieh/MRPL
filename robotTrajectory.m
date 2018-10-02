%% Robot Trajectory

classdef robotTrajectory < handle
    properties
        refcontrol
        numsamples
    end
    methods 
        function obj = robotTrajectory(refcontrol, numsamples)
            obj.refcontrol = refcontrol;
            obj.numsamples = numsamples;
        end
        
        function [tArr, dArr, vArr, pArr] = generateSamples(numsamples)
            V = 0;
            w = 0;
            dist = 0;
            angle = pi/2;
            x = 0;
            y = 0;
            p = [x, y, angle];
            tArr = [];
            dArr = [];
            vArr = [];
            pArr = [];
            dt = figure8ReferenceControl.refcontrol.totalTime / numsamples;
            for i = 1:(numsamples)
                ti = (i-1)*dt;
                tArr = [tArr, ti]; %time update
                [V, w] = figure8ReferenceControl.computeControl(obj.refcontrol, ti);
                vArr = [vArr V]; %velocity update
                dist = dist + (V * dt);
                dArr = [dArr, dist]; %distance update
                angle = angle + w*dt;
                x = V*dt*sin(angle);
                y = V*dt*cos(angle);
                p = [x, y, angle];
                pArr = [pArr, p]; %position update
            end
        end
    end
end