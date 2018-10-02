%%
classdef figure8ReferenceControl
    properties(Constant)
        vt = 0.2;
        sf = 1;
        tf = 1/0.2;
        ktheta = 2*pi/1;
        kk = 15.1084;
    end
    properties(Access = public)
        ks
        kv
        Tf
        totalTime
        tPause
    end
    methods
        function obj = figure8ReferenceControl(Ks,Kv,tPause)

            obj.tPause = tPause;
            obj.ks = Ks;
            obj.kv = Kv; 
            obj.Tf = (obj.ks / obj.kv)*obj.tf;
            start = 0;
            T = 0;
            totalTime = 2 * obj.tPause + obj.Tf;
            obj.totalTime = totalTime;
            tdelay = 0.2;
        end
        function [V, w] = computeControl(obj,timeNow)
        % Return the linear and angular velocity that the robot
            if ((timeNow < obj.tPause) || (obj.totalTime - timeNow < obj.tPause) || (timeNow > obj.totalTime))
                V = 0; w = 0;
            else 
                st = obj.vt*timeNow;
                k = (obj.kk/obj.ks)*sin(obj.ktheta*st);
                V = obj.kv*obj.vt;
                w = k*V;
             end
        end
        
        function duration = getTrajectoryDuration(obj)
         % Return the total time required for motion and for the
         % initial and terminal pauses.
            duration = obj.totalTime;
        end 
    end
    end