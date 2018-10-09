classdef figure8ReferenceControl < handle
    properties(Constant)
    end
    properties(Access = private)
    end
    properties(Access = public)
        sf; 
        kk;
        ks;
        kv;
        timeDelay;
        vt; 
        tf;
        ktheta;
        st;
        k;
        wt;
        Tf;
        V;
    end
    methods(Static = true)
        function obj = figure8ReferenceControl(Ks, Kv, tPause)
            %Construct a figure 8 trajectory. It will not start until
            %tPause has elapsed and it will stay at zero for tPause
            %afterwards. 
            %Kv scales velocity up when >1
            %Ks scales the size of curve up
            
            %v = 0.1; w = kw*t; kw = 1/8
            %vr = v + W/2*angularVelocity
            %vl = v - W/2*angularVelocity 
            obj.sf = 1; 
            obj.kk = 15.1084;
            obj.ks = Ks;
            obj.kv = Kv;
            obj.vt = 0.2;
            obj.ktheta = 2*pi/obj.sf;
            obj.timeDelay = tPause;
            if(Kv > 1)
                obj.vt = obj.vt*Kv;
            end
            obj.tf = obj.sf/obj.vt;
            obj.ktheta = 2*pi/obj.sf;
            obj.Tf = (obj.ks/obj.kv)*obj.tf;
        end
        function [V, w] = computeControl(obj, timeNow)
            %Return the linear and angular velocity that the
            %robot should be executing at the time timeNow. Any zero
            %velocity pauses specified in the constructure are implemented
            %here
            t = (obj.kv/obj.ks)*(timeNow-obj.timeDelay);
            obj.st = obj.vt*t;
            obj.k = obj.kk/obj.ks*sin(obj.ktheta*obj.st);
            V = obj.kv*obj.vt;
            w = obj.k*V;
        end
        function duration = getTrajectoryDuration(obj)
            %Return the total time required for motion and for the initial
            %and terminal pauses. 
            duration = obj.Tf + 2*obj.timeDelay;
        end
    end
end