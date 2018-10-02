classdef trapezoidalStepReferenceControl < handle
    properties(Constant)
        wbase = .088;
        vt = 0.2; 
        sf = 1; 
        tf = sf/vt;
        ktheta = (2*pi)/sf; 
        kk = 15.1084;
    end
    properties(Access = private)
        dist;
        amax;
        vmax;
        tramp;
        Tf;
    end
    methods(Access = public) 
        function obj = trapezoidalStepReferenceControl (d, AccCap, VelCap, tPause)
            obj.stall = tPause; 
            obj.dist = d;
            obj.amax = AccCap;
            obj.vmax = VelCap;
            obj.Tf = ((obj.dist + obj.vmax^2)/obj.amax)/obj.vmax;
            obj.tramp = obj.vmax/obj.amax;
        end
        function uref = computeControl(obj, timeNow)
            if (timeNow <= tPause) || (timeNow >= obj.Tf + obj.stall)
                uref = 0
            elseif (timeNow < obj.tramp)
                    uref = amax * t;
            elseif (obj.Tf - timeNow) < obj.tramp)
                uref = amax * (obj.Tf - timeNow)
            elseif (obj.tramp < timeNow && timeNow < (obj.Tf - obj.tramp))
                uref = vmax;
            else
                uref = 0;
            end
        end
    end
end