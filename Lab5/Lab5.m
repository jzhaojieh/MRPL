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
pArr = [];
dArr = [];
eArr = [];
thArr = [];
x = 0;
y = 0;
theta = 0;

enable = 1;
dref = 0;
ddelay = 0;
start = tic;
t = toc(start);
oldt = t;
refcontrol = figure8ReferenceControl(1, 1, 1);
traj = robotTrajectory(refcontrol, 10000);
traj.generateSamples();
% tArr = traj.tArr;
dArr = traj.dArr;
pArr = traj.pArr;
control = controller(traj);
tf = refcontrol.totalTime;
while (t < tf && abs(goal - (encoderCur - encoderStart)) > 0.0001)
    oldt = t;
    t = toc(start);
    dt = t - oldt;
    
    
    encoderx = robot.encoders.LatestMessage.Vector.X;
    encodery = robot.encoders.LatestMessage.Vector.Y;
    encoderCur = (encoderx + encodery) / 2;
    d = encoderCur - encoderStart;
    
    Vref = traj.getVelForTime(t);
    wref = traj.getWForTime(t);
    [Vpid, wpid, derror, therror] = control.pid(t, [x, y, theta]); 
    
    eArr = [eArr sqrt(derror(1)^2 + derror(2)^2)];
    thArr = [thArr therror];
    tArr = [tArr t];
    [vlref, vrref] = robotModel.VwTovlvr(Vref, wref);
    [vlpid, vrpid] = robotModel.VwTovlvr(Vpid, wpid);
    ul = enable*vlpid + vlref;
    ur = enable*vrpid + vrref;
    
    if (isnan(ul) || isnan(ur))
        Vref = 0;
        Vpid = 0;
        wref = 0;
        wpid = 0;
        ul = 0;
        ur = 0;
    end
    
    x = x + (Vref + enable*Vpid)*dt*sin(theta+(wref+enable*wpid)*dt);
    y = y + (Vref + enable*Vpid)*dt*cos(theta+(wref+enable*wpid)*dt);
    p = [x; y; theta+(wref+enable*wpid)*dt];
    
    robot.sendVelocity(ul, ur);
    
    pause(.05);
end
plot(tArr, eArr, tArr, thArr);
robot.stop();
robot.shutdown();