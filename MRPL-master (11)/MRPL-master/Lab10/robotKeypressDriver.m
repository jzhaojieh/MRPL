classdef robotKeypressDriver < handle
    %robotKeypressDriver Creates a keyboard event handler and then lets
    % the user drive the robot with the arrow keys.
    properties(Constant)
        linVel = 0.02;
        angVel = 0.06; % 0.006 / 0.1 (for W)
        
    end
    properties(Access = private)
        fh=[];
    end
    properties(Access = public)
    end
    methods(Static = true)
        function drive(robot,vGain)
        % drive the robot
            Vmax = robotKeypressDriver.linVel*vGain;
            dV = robotKeypressDriver.angVel*robotModel.W*vGain;
            k = waitforbuttonpress;
            value = double(get(gcf,'CurrentCharacter'));
            if(value == 30)
                disp('up');
                robot.sendVelocity(Vmax,Vmax);
            elseif(value == 31)
                disp('down');
                robot.sendVelocity(-Vmax,-Vmax);
            elseif(value == 28)
                disp('left');
                robot.sendVelocity(Vmax,Vmax+dV);
            elseif(value == 29)
                disp('right');
                robot.sendVelocity(Vmax+dV,Vmax);
            elseif(value == 115)
                disp('stop');
                robot.sendVelocity(0.0,0.0);
            end
        end
    end
    methods(Access = private) 
    end
    methods(Access = public)
        function obj = robotKeypressDriver()
        % create a robotKeypressDriver for the figure handle
        % normally you call this with gcf for fh
        obj.fh = figure;
        set(obj.fh,'KeyPressFcn',@keyboardEventListener);
        end
    end
end
function keyboardEventListener(~,event)
%keyboardEventListener Invoked when a keyboard character is
%     pressed.
    global keypressFrame;
    global keypressDataReady;
    global keypressKey;
    keypressFrame = keypressFrame + 1;
    keypressDataReady = 1;
    keypressKey = event.Key;
    disp(event.Key);
end
function res = pollKeyboard()
    %pollKeyboard Waits until the callback says there is new data.
    % This routine is useful when you want to be able to capture
    % and respond to a keypress as soon as it arrives.
    % To use this, execute the following line:
    % kh = event.listener(gcf,'KeyPressFcn',@keyboardEventListener);
    % before calling this function.
    global keypressDataReady;
    global keypressKey;
    keyboardDataReadyLast = keypressDataReady;
    keypressDataReady = 0;
    if(keyboardDataReadyLast)
        res = keypressKey;
        disp('gotOne');
    else
        res = false;
    end
end