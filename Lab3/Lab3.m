%% Lab 3 Model for Simulation

global robot
global curval
global tstamp
global encoderTimeStamp
robot = raspbot('Raspbot-07');
pause(2);
preval = false;
curval = false;
tstamp = 0;
global newenc;


% initialize constants
wbase = .088;
vt = 0.2; sf = 1; tf = sf/vt;
ktheta = (2*pi)/sf; kk = 15.1084;
ks = 2; Tf = ks * tf;


% used to store data
xArr = []; yArr = [];
thArr = [];


robot.encoders.NewMessageFcn=@encoderEventListener;

prevx = robot.encoders.LatestMessage.Vector.X;
prevy = robot.encoders.LatestMessage.Vector.Y;


t = 0;
oldenc = [0,0,0];
global myplot;
x = 0;
y = 0;
th = 0;
newenc = oldenc;


while (t < Tf)
    
    preval = curval;
    st = vt*t/ks;
    angle = ktheta * st;

    dt = newenc(3) - oldenc(3);
    
    k = (kk / ks)*sin(angle);
    omegat = k * vt;
    vr = vt + (0.044) * omegat;
    vl = vt - (0.044) * omegat;
    
    robot.sendVelocity(vl, vr)
    
    if dt ~= 0
        encoderx = newenc(1);
        encodery = newenc(2);
        dx = encoderx - prevx;
        dy = encodery - prevy;
        prevx = encoderx;
        prevy = encodery;
        vlactual = dx/dt;
        vractual = dy/dt;
        Vactual = (vlactual+vractual)/2;
        omegaActual = (vractual-vlactual)/.088;
        th = th + omegaActual*dt;
        x = x + Vactual*cos(th)*dt;
        y = y + Vactual*sin(th)*dt;
        xArr = [xArr, x];
        yArr = [yArr, y];
        plot(yArr, xArr);
        title("Robot Encoder Data");
        xlabel('X(m)');
        ylabel('Y(m)');
    end
    
    oldenc = newenc;
    t = t + dt;
    pause(0.05);
end

robot.encoders.NewMessageFcn = [];
robot.sendVelocity(0,0);
robot.stop()
robot.shutdown()


function encoderEventListener(handle,event)
    global newenc
    global robot
    global curval
    global tstamp
    global encoderTimeStamp
    curval = ~curval;
    if tstamp == 0
        tstamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1e9;
    end
    encoderTimeStamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1e9 - tstamp;
    e = [robot.encoders.LatestMessage.Vector.X, robot.encoders.LatestMessage.Vector.Y, encoderTimeStamp];
    newenc = e;
end