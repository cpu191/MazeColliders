clear all
path = pwd
addpath(genpath([path,'\Toolboxes\']))
% run startMobileRoboticsSimulationToolbox.m
sensorAngle = 0*pi/180; %% Sensor scan angle in radian
scanDensity = 1; %% Amount of beam emited
sensorRange = 5; %% Max range of sensor
originalPose = [3; 4; 0]; %% Original robot position
port = serialportlist("available") %% List the available Serial ports
SerialPort = "COM4"; %% Change to the port connected to the Arduino
BaudRate = 9600;   %% Communication baud rate
odometer = 0; %% Distance travelled
velocity = 0;
velocity_h=NaN;
%% Innitialize Serial Communication
arduinoObj = serialport(SerialPort,BaudRate);
configureTerminator(arduinoObj,"CR/LF");
flush(arduinoObj);
arduinoObj.Timeout = 60; % Serial connection not reveive data within 60 second will stop automatically

%% Innitialize the Environment
viz = Visualizer2D;
viz.showTrajectory = false;
%% Attemp to create map

% Load map from .png pixel drawing (256x256 pixel)
image = imread('map1.png');
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;
map = binaryOccupancyMap(bwimage,10);


viz.mapName = 'map';
pose = [3; 4; 0];
viz(pose)
release(viz);
lidar = LidarSensor;
lidar.scanAngles = linspace(sensorAngle-pi/180,sensorAngle+pi/180,scanDensity);
ranges=NaN;
lidar.maxRange = sensorRange;
attachLidarSensor(viz,lidar);

while 1
    
    buffer = arduinoObj.readline;
    if ~isempty(buffer)
        switch buffer
            case "CLOSE"
                arduinoObj = [];
                break
            otherwise
                cmd = strsplit(buffer,'_'); % Splitting command into sectors
                if cmd(1) == 'CMD'
                    switch cmd(2)
                        case "ACT"          % Actuate Command
                            switch cmd(3)
                                case "LAT"                           % CMD_ACT_LAT_*Direction*_*DistanceValue* - Forward and backwards movement, Direction 1= forward, 0 =backward (m/s)
                                    direction = str2double(cmd(4));
                                    distance = str2double(cmd(5));
                                    [viz,pose,odometer,lidar,velocity_h,velocity]= moveStep(viz,pose,distance,direction,odometer,lidar,velocity_h,velocity,map);
                                case "ROT"                           % CMD_ACT_ROT_*Direction*_*Angle* - CW and CCW rotation,  Direction 1= CW, 0=CCW, Angle= rotation angle value
                                    direction = str2double(cmd(4));
                                    angle = str2double(cmd(5))*pi/180;
                                    switch direction
                                        case 1
                                            pose = pose - [0; 0; angle];
                                            ranges = lidar(pose);
                                            viz(pose,ranges);
                                            pause(0.01)
                                        case 0
                                            pose = pose + [0; 0; angle];
                                            ranges = lidar(pose);
                                            viz(pose,ranges)
                                            pause(0.01)
                                        otherwise
                                            disp('WRONG DIRECTION INPUT')
                                    end
                                    if pose(3) >= 2*pi
                                        pose(3) = pose(3) - 2*pi;
                                    end
                                    if pose(3) < 0
                                        pose(3) = pose(3) + 2*pi;
                                    end
                                    
                                case "SPEED"                         % CMD_ACT_SPEED_*SpeedVal* - Set speed for robot, speedval=integer for speed (m/s)
                                    %velocity  = cmd(4);
                                    
                            end
                        case "SEN"          % Sensor Command
                            switch cmd(3)
                                case "US"                            % CMD_SEN_US - Function returns distance readings of the ultrasound sensor
                                    SensorUS = ranges
                                case "IR"                            % CMD_SEN_IR - Function returns readings of the laser sensor
                                    if ~isempty(ranges)
                                        IRreading = ranges
                                    end
                                case "ROT"                           % CMD_SEN_ROT_*Angle* - This will move the sensor to the specific position 0 is forward, 90 is right, 180 is back, 270 is left
                                    while cmd(4)> 360
                                        cmd(4) = cmd(4)- 360;
                                    end
                                    while cmd(4)<0
                                        cmd(4) = cmd(4) + 360;
                                    end
                                    
                                    newSensorAngle = str2double(cmd(4))*pi/180; %% Sensor scan angle in radian
                                    if newSensorAngle > sensorAngle && (newSensorAngle-sensorAngle) < pi
                                        for i=0 : 1 : newSensorAngle
                                            release(viz)
                                            lidar.scanAngles = linspace(sensorAngle-pi/180,sensorAngle+pi/180,scanDensity);
                                            attachLidarSensor(viz,lidar);
                                            ranges = lidar(pose);
                                            viz(pose,ranges);
                                        end
                                    else
                                        %??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
                                    end
                                    
                                    
                                case "CHECK"            % CMD_SEN_CHECK - This will return the current rotation on the sensor rotation 0-360
                                    sensorAngle;
                                case "VEL"              % CMD_SEN_VEL - Return current Velocity of the robot
                                    velocity;
                                case "GYRO"             % CMD_SEN_GYRO - Return current orientation of the robot
                                    Robot_Orientation = pose(3)*180/pi
                                case "DIST"             % CMD_SEN_DIST - Return current orientation of the robot
                                    Odometer = odometer
                            end
                            
                    end
                end
        end
    end
end

function [viz,pose,odometer,lidar,velocity_h,velocity]= moveStep(viz,pose,distance,direction,odometer,lidar,velocity_h,velocity,map)
try delete(velocity_h);end
switch direction
    case 1
        velocity_h = text(17,27,sprintf('Velocity(m/s)=%f',1.5),'FontSize',7);
        velocity = 1.5;
        if distance < 2
            for i = 0:distance/25: distance
                nextPose = pose + [distance/25*cos(pose(3));distance/25*sin(pose(3));0];
                if getOccupancy(map,[nextPose(1) nextPose(2)]) == 1
                    disp('Wall Stop')
                    return
                else
                    pose = nextPose;
                end
                ranges = lidar(pose);
                viz(pose,ranges)
                pause(0.01)
            end
        else
            for i = 0:distance/50: distance
                nextPose = pose + [distance/50*cos(pose(3));distance/50*sin(pose(3));0];
                if getOccupancy(map,[nextPose(1) nextPose(2)]) == 1
                    disp('Wall Stop')
                    return
                else
                    pose = nextPose;
                end
                ranges = lidar(pose);
                velocity = 1.5;
                viz(pose,ranges)
                pause(0.01)
            end
        end
        delete(velocity_h);
        velocity_h = text(17,27,sprintf('Velocity(m/s)=%f',0),'FontSize',7);
        velocity=0;
    case 0
        velocity_h = text(17,27,sprintf('Velocity(m/s)=%f',-1.5),'FontSize',7);
        velocity = -1.5;
        if distance < 2
            for i = 0:distance/25: distance
                nextPose = pose - [distance/25*cos(pose(3));distance/25*sin(pose(3));0];
                if getOccupancy(map,[nextPose(1) nextPose(2)]) == 1
                    disp('Wall Stop')
                    return
                else
                    pose = nextPose;
                end
                ranges = lidar(pose);
                velocity = -1.5;
                viz(pose,ranges)
                pause(0.02)
            end
        else
            for i = 0:distance/50: distance
                nextPose = pose - [distance/50*cos(pose(3));distance/50*sin(pose(3));0];
                if getOccupancy(map,[nextPose(1) nextPose(2)]) == 1
                    disp('Wall Stop')
                    return
                else
                    pose = nextPose;
                end
                ranges = lidar(pose);
                velocity = -1.5;
                viz(pose,ranges)
                velocity_h = text(17,27,sprintf('Velocity=%f',velocity,'m/s'),'FontSize',7);
                
                pause(0.01)
            end
        end
        delete(velocity_h);
        velocity_h = text(17,27,sprintf('Velocity(m/s)=%f',0),'FontSize',7);
        velocity=0;
    otherwise
        disp('WRONG DIRECTION INPUT')
end
odometer = odometer + sqrt(distance*cos(pose(3))*distance*cos(pose(3))+distance*sin(pose(3))*distance*sin(pose(3)));
end
