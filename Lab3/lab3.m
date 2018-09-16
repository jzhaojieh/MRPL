%% Lab 3 Model for Simulation
robot = raspbot('sim');
curval = false;

% initialize constants
wbase = .088;
vt = 0.2; sf = 1; tf = sf/vt;
ktheta = (2*pi)/sf; kk = 15.1084;
ks = 3; Tf = ks * tf;

% ####### FOR SIM ##############
start = tic;
t = toc(start);
% ##############################

% used to store data
vlArr = []; vrArr = []; dtArr = [];
xArr = []; yArr = []; VArr = [];

% robot.encoders.NewMessageFcn=@encoderEventListener;

% prevx = robot.encoders.LatestMessage.Vector.X;
% prevy = robot.encoders.LatestMessage.Vector.Y;
% ##### Not Sim ##################
t = 0;
% ################################
oldenc = getEncoders();
% global myplot;
x = 0;
y = 0;
th = 0;
% myplot = plot(xArr, yArr, 'b-');

while (t < Tf)
    newenc = getEncoders();
%     encoderx = robot.encoders.LatestMessage.Vector.X;
%     encodery = robot.encoders.LatestMessage.Vector.Y;
%     dx = encoderx - prevx;
%     dy = encodery - prevy;
%     prevx = encoderx;
%     prevy = encodery;
    dt = newenc(3) - oldenc(3);
    if dt == 0
        continue
    end
    
    st = vt*t/ks;
    angle = ktheta * st;
    k = (kk / ks)*sin(angle)
    omegat = k * vt;
    vr = vt + (0.044) * omegat;
    vl = vt - (0.044) * omegat;
    vlArr = [vlArr vl];
    vrArr = [vrArr vr];

    robot.sendVelocity(vl, vr);
    T = toc(start);
    dt = (T - t);
%     plot(yArr, xArr);
%     vl = (newenc(1) - oldenc(1))/dt;
%     vr = (newenc(2) - oldenc(2))/dt;
    omega = (vr-vl)/wbase;
    V = (vr+vl)/2;
     
    th = th + omega * dt;
    x = x + V*cos(th)*dt;
    y = y + V*sin(th)*dt;
    xArr = [xArr, x];
    yArr = [yArr, y];
%     VArr = [vArr V];
    plot(xArr, yArr);
    title("Robot Encoder Data");
%     set(myPlot, 'xdata', [get(myPlot,'xdata') x], 'ydata', [get(myPlot,'ydata') y]);
    xlabel('X(m)');
    ylabel('Y(m)');
%     dtArr = [dtArr dt];
%     
%     plot(vlArr, dtArr)
    t = T;
    pause(0.05);

end


robot.encoders.NewMessageFcn = [];
robot.sendVelocity(0,0);
robot.shutdown()


function encoderEventListener(handle,event)
end

function e = getEncoders()
%      tstamp = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
%      e = [robot.encoders.LatestMessage.Vector.X, robot.encoders.LatestMessage.Vector.Y, tstamp];
end