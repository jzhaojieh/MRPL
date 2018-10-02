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
        % Construct a figure 8 trajectory. It will not start until
        % tPause has elapsed and it will stay at zero for tPause
        % afterwards. Kv scales velocity up when > 1 and Ks scales
        % the size of the curve itself up.
%             obj.vt = 0.2;
%             obj.sf = 1;
%             obj.tf = obj.sf/obj.vt; 
        
%             obj.ktheta = 2*pi/sf;
%             obj.kk = 15.1084;
            obj.tPause = tPause;
            obj.ks = Ks;
            obj.kv = Kv; 
            obj.Tf = (obj.ks / obj.kv)*obj.tf;
            start = 0;
            T = 0;
            totalTime = 2 * obj.tPause + obj.Tf;
            obj.totalTime = totalTime;
            tdelay = 0.2;
%             firstIteration = false;
%             while (T < totalTime)
%                 if (firstIteration == false)
%                     start = tic();
%                     tstart = toc(start);
%                     firstIteration = True;
%                 end
%                 t = (Ks/Kv) * (T - tdelay);
%                 [V w] = figure8ReferenceControl.computeControl(obj, t);
%                 [vl vr] = robotModel.VwTovlvr(V, w);
%                 pause(0.01);
%                 T = toc(start);
%                 obj.T = [obj.T T];
%             end
        end
        function [V w] = computeControl(obj,timeNow)
        % Return the linear and angular velocity that the robot
            totalTime = figure8ReferenceControl.getTrajectoryDuration(obj);
            if ((timeNow < obj.tPause) || (totalTime - timeNow < obj.tPause) || (timeNow > totalTime))
                V = 0; w = 0;
            else 
                st = obj.vt*timeNow;
                k = (obj.kk/obj.ks)*sin(obj.ktheta*st);
                V = obj.kv*obj.v;
                w = k*V;
             end
        end
        
        function duration = getTrajectoryDuration(obj)
         % Return the total time required for motion and for the
         % initial and terminal pauses.
            duration = 2 * obj.tPause + obj.Tf;
        end 
    end
    end