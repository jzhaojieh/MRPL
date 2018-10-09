%%Lab 5 Trajectory Tracking
% robot = raspbot('Raspbot-7');
robot = raspbot('sim');

start = tic;
t = toc(start);

%=========TOGGLE PID========================
enable = 0;
%===========================================
%==========INITIALIZATION====================

refCon = figure8ReferenceControl(3, 1, 1);
robTraj = robotTrajectory(1000, refCon);
rob = robotModel();
trajFollow = trajectoryFollower(rob, robTraj);
%-------this might mess up-------------------
lRead = robot.encoders.LatestMessage.Vector.X;
rRead = robot.encoders.LatestMessage.Vector.Y;
pid = controller(rob, robTraj, lRead, rRead);
%--------------------------------------------
robTraj.generateSamples(robTraj);
timeFinal = 100;
%============================================
while(t < timeFinal)
    t = toc(start);
    [vlFF, vrFF] = trajFollow.feedForwardVel(trajFollow, t);
    [vlFB, vrFB] = pid.giveError(pid, lRead, rRead, t);
    vl = enable*vlFB + vlFF;
    vr = enable*vrFB + vrFF;
    robot.sendVelocity(vl, vr);
    pause(.05);
    timeFinal = refCon.getTrajectoryDuration(refCon);
end
robot.stop();
robot.shutdown();