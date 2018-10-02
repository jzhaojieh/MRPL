classdef Figure8ReferenceControl < handle
    properties(Constant)
        wbase = .088;
        vt = 0.2; 
        sf = 1; 
        tf = sf/vt;
        ktheta = (2*pi)/sf; 
        kk = 15.1084;
    end
    properties(Access = private)
        ks; 
        kv;
        stall;
        Tf;
    end
    methods(Access = public) 
        function obj = Figure8ReferenceControl (Ks, Kv, tPause)
            obj.ks = Ks;
            obj.kv = Kv;
            obj.stall = tPause; 
            obj.Tf = (obj.ks/obj.kv)/obj.tf;
        end
        function [V w] = computeControl(obj, timeNow)
            if (timeNow <= tPause)
                V = 0;
                w = 0;
            else if (timeNow >= obj.Tf + obj.stall)
                    V = 0;
                    w = 0;
                else
                    curV = kv*vt;
                    curW = kt*curV
                    V = curV;
                    w = curW;
                end
            end
        end
        function duration = getTrajectoryDuration(obj)
            duration = obj.Tf + 2 * obj.stall;
        end
    end
end
        

