classdef figure8ReferenceControl < handle
    properties(Constant)
        sf = 1; 
        kk = 15.1084;
    end
    properties(Access = private)
    end
    properties(Access = public)
        ks;
        kv;
        timeDelay;
        vt = 0.2; 
        tf;
        ktheta = 2*pi/sf;
        st = vt*t;
        k = kk/ks*sin(ktheta*st);
        wt = k*vt;
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
            object.ks = Ks;
            object.kv = Kv;
            object.timeDelay = tPause;
            if(Kv > 1)
                object.vt = object.vt*Kv;
            end
            object.tf = object.sf/object.vt;
            object.ktheta = 2*pi/object.sf;
            object.Tf = (object.ks/object.kv)*object.tf;
        end
        function [V, w] = computeControl(obj, timeNow)
            %Return the linear and angular velocity that the
            %robot should be executing at the time timeNow. Any zero
            %velocity pauses specified in the constructure are implemented
            %here
            t = (object.kv/object.ks)*(timeNow-object.timeDelay);
            object.st = vt*t;
            object.k = object.kk/object.ks*sin(object.ktheta*object.st);
            V = object.kv*object.vt;
            w = object.k*object.V;
        end
        function duration = getTrajectoryDuration(obj)
            %Return the total time required for motion and for the initial
            %and terminal pauses. 
            duration = object.Tf + 2*object.timeDelay;
        end
    end