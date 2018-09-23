%% Lab 4 Feedforward Control
% robot = raspbot('sim');
robot = raspbot('Raspbot-07');
goal = 0.5;
encoderStart = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
encoderCur = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
e = (goal)-(encoderCur-encoderStart);

kp = 10; kd = 0; ki = 0.0;
eI = 0;
tArr = [];
start = tic;
t = toc(start);
upid = 0;
errArr = [];
fprintf('start is equal to %s\n',num2str(tic))
fprintf('e is equal to %s\n',num2str(e))
while (t < 6 && abs(e) > 0.00001)
    olde = e;
    oldt = t;
    encoderCur = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
    e = goal - (encoderCur - encoderStart);
    t = toc(start);
    dt = t - oldt;
    eD = (e - olde)/dt;
    eI = eI + (e * dt);
%     sign = sign(eI);
    if abs(eI) > 0.1
        if eI > 0
            eI = 0.1;
        else
            eI = -1 * 0.1;
        end
    end
    upid = (e * kp) + (eD * kd) + (eI * ki);
    if abs(upid) > 0.3
        if upid > 0
            upid = 0.3;
        else
            upid = -1 * 0.3;
        end
    end
    robot.sendVelocity(upid, upid);
    pause(0.1)
    errArr = [errArr e];
    tArr = [tArr t];
    plot(tArr, errArr);
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
    if t < 0
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