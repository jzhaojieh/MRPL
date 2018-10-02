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
        
        function res = generateSamples(numsamples)
            V = 0;
            w = 0;
            si = 0;
            d = 0;
            p = [0, 0, 0];
            res = [];
            tArr = [];
            dArr = [];
            vArr = [];
            pArr = [];
            dt = figure8ReferenceControl.refcontrol.totalTime / numsamples;
            for i = 1:(numsamples)
                ti = (i-1)*dt;
                [V w] = figure8ReferenceControl.computeControl(obj.refcontrol, ti);
                vArr = [vArr V];
                si = si + (v * dt);
                
            end
        end
    end
end