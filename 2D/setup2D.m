clear all
scanAngle = pi/3; %% Input sensor scan angle
scanDensity = 20; %% Amount of beam emited
originalPose = [3; 4; 0]; %% Original robot position


port = serialportlist("available") %% List the available Serial ports
SerialPort = "COM4"; %% Change to the port connected to the Arduino
BaudRate = 9600;   %% Communication baud rate

%% Innitialize Serial Communication
arduinoObj = serialport(SerialPort,BaudRate);
configureTerminator(arduinoObj,"CR/LF");
flush(arduinoObj);

%% Innitialize the Environment
viz = Visualizer2D;
viz.showTrajectory = false;
load exampleMap.mat
viz.mapName = 'map';


pose = [3; 4; 0];
viz(pose)
release(viz); 
lidar = LidarSensor;
lidar.scanAngles = linspace(-scanAngle/2,scanAngle/2,scanDensity);
attachLidarSensor(viz,lidar);

while 1
    
    buffer = arduinoObj.readline;
    
    switch buffer
        case "UP"
            pose = pose + [0.2*cos(pose(3));0.2*sin(pose(3));0];
            ranges = lidar(pose)
            viz(pose,ranges)
            pause(0.01)
        case "DOWN"
            pose = pose - [0.1*cos(pose(3));0.1*sin(pose(3));0];
            ranges = lidar(pose)
            viz(pose,ranges)
            pause(0.01)
        case "RIGHT"
            pose = pose - [0; 0; pi/70];
            ranges = lidar(pose)
            viz(pose,ranges)
            pause(0.01)
        case "LEFT"
            pose = pose + [0; 0; pi/70];
            ranges = lidar(pose)
            viz(pose,ranges)
            pause(0.01)
    end
end 

