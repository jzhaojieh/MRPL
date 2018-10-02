%% Lab 5 Trajectory Tracking
% robot = raspbot('Raspbot-07');

robot = raspbot('sim');
global eold;
global eint;
global amax;
global vmax;
global goal;
global tdelay;
global tf;

eold = 0;
eint = 0;
goal = .9144;
vmax = .05;
amax = 3*vmax;
tdelay = 0.2;

encoderx = robot.encoders.LatestMessage.Vector.X;
encodery = robot.encoders.LatestMessage.Vector.Y;
encoderStart = (encoderx + encodery) / 2;
encoderCur = encoderStart;
tArr = [];
udArr = [];
dArr = [];

enable = 1;
dref = 0;
ddelay = 0;
tf = 50;
start = tic;
t = toc(start);
oldt = t;
refcontrol = figure8ReferenceControl([1, 1, 1]);
traj = robotTrajectory(refcontrol, 500);
control = controller(traj);
while (t < (tf + 1) && abs(goal - (encoderCur - encoderStart)) > 0.0001)
    oldt = t;
    t = toc(start);
    dt = t - oldt;
    
    
    encoderx = robot.encoders.LatestMessage.Vector.X;
    encodery = robot.encoders.LatestMessage.Vector.Y;
    encoderCur = (encoderx + encodery) / 2;
    d = encoderCur - encoderStart;
    
    Vref = traj.getVelForTime(t);
    wref = traj.getWForTime(t);
    [Vpid, wpid] = control.pid(t, traj.getPoseForTime(t-tdelay)); 
    [vlref, vrref] = robotModel.VwTovlvr(Vref, wref);
    [vlpid, vrpid] = robotModel.VwTovlvr(Vref, wref);
    
    ul = enable*vlpid + vlref;
    ur = enable*vrpid + vrref;
    
    robot.sendVelocity(ul, ur);
    
    tArr = [tArr t];
    udArr = [udArr dref];
    dArr = [dArr d];
    plot(tArr, udArr, tArr, dArr);
    pause(.05);
end


function upid = getpid(d, dt, ddelay)
global eold;
global eint;

kp = 4;
kd = .01;
ki = .005;

enow = ddelay-d;   
    edir = abs((enow - eold)/dt);
    eint = eint + (enow * dt);
    if abs(eint) > 0.1
        if eint > 0
            eint = 0.1;
        else
            eint = -1 * 0.1;
        end
    end
    upid = (enow * kp) + (edir * kd) + (eint * ki);
    if abs(upid) > 0.3
        if upid > 0
            upid = 0.3;
        else
            upid = -1 * 0.3;
        end
    end
    eold = enow;
end

function uref = trapezoidalVelocityProfile( t , amax, vmax, dist, sgn)
% Returns the velocity command of a trapezoidal profile of maximum
% acceleration amax and maximum velocity vmax whose ramps are of
% duration tf. Sgn is the sign of the desired velocities.
% Returns 0 if t is negative. 
global tf;
    tramp = vmax / amax;
    sf = dist;
    tf = (sf + vmax^2/amax)/vmax;
    
    if t < 0 || t >= tf
        uref = 0;
    elseif t < tramp
        uref = amax * t;
    elseif (tf - t) < tramp
        uref = amax * (tf - t);
    elseif tramp < t && t < (tf - tramp)
        uref = vmax;
    else
        uref = 0;
    end
    uref = uref * sgn;
end