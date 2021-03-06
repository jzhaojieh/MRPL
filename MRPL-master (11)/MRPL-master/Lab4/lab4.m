%% Lab 4 Feedforward Control
% robot = raspbot('sim');
robot = raspbot('Raspbot-07');
goal = .9144;
encoderStart = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
encoderCur = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
e = (goal)-(encoderCur-encoderStart);

enable = 1;
tdelay = 0.2;
kp = 3; kd = 0.02; ki = 0;
eI = 0;
tArr = [];
drefArr = [];
adArr = [];
start = tic;
t = toc(start);
upid = 0;
dref = 0;

errArr = [];
e2Arr = [];
upidArr = [];
e2 = 0;
vmax = .25;
amax = 3 * .25;
uref = 0;
udelay = 0;

fprintf('start is equal to %s\n',num2str(tic))
fprintf('e is equal to %s\n',num2str(e))
tf = (goal + vmax^2/amax)/vmax;
while (t < (5.33 + tdelay) && abs(goal - (encoderCur - encoderStart)) > 0.00001)
    olde = e2;
    oldt = t;
    
    t = toc(start);
    sgn = sign(e);
    
    
    uref = trapezoidalVelocityProfile( t , amax , vmax , goal, sgn);
    udelay = trapezoidalVelocityProfile( t - tdelay, amax , vmax , goal, sgn);
    dt = t - oldt;
    ddref = ddref + uref * dt;
    dref = dref + udelay * dt;
    
    
    encoderCur = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
    e = goal - (encoderCur - encoderStart);
    
    
%     cdref = udelay * dt;
    cd = (encoderCur - encoderStart);
    e2 = -cd + dref;
    
    eD = (e2 - olde)/dt;
    eI = eI + (e2 * dt);
%     sign = sign(eI);
    if abs(eI) > 0.1
        if eI > 0
            eI = 0.1;
        else
            eI = -1 * 0.1;
        end
    end
    upid = (e2 * kp) + (eD * kd) + (eI * ki);
    if abs(upid) > 0.3
        if upid > 0
            upid = 0.3;
        else
            upid = -1 * 0.3;
        end
    end
%     dref = dref + uref * dt;
  
    actualD = encoderCur - encoderStart;
    u = enable*upid + uref;
    
    robot.sendVelocity(u, u);
    upidArr = [upidArr upid];
    pause(0.001)
    errArr = [errArr e];
    e2Arr = [e2Arr e2];
    tArr = [tArr t];
    drefArr = [drefArr dref];
    adArr = [adArr actualD];
    plot(tArr, drefArr, tArr, adArr);
end
% 
% start = tic;
% t = toc(start);
% encoderCur = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
% e = (goal)-(encoderCur-encoderStart);

robot.sendVelocity(0,0);
robot.shutdown()

function uref = trapezoidalVelocityProfile( t , amax, vmax, dist, sgn)
% Returns the velocity command of a trapezoidal profile of maximum
% acceleration amax and maximum velocity vmax whose ramps are of
% duration tf. Sgn is the sign of the desired velocities.
% Returns 0 if t is negative. 
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