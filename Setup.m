clear
clf
global robotPosition
port = serialportlist("available")
SerialPort = "COM4"; %% Change to the port connected to the Arduino
BaudRate = 9600;   %% Communication baud rate
robotPosition = transl(0,0,0.1);  % robot initial position is (0,0)

%% Load floor
floorPose=transl(0,0,0);
[fFloor,vFloor,dataFloor] = plyread('floor.ply','tri');
floorVertexColours = [dataFloor.vertex.red,dataFloor.vertex.green,dataFloor.vertex.blue] / 255;
floor_h = trisurf(fFloor,vFloor(:,1),vFloor(:,2),vFloor(:,3),'FaceVertexCData',floorVertexColours,'EdgeColor','interp','EdgeLighting','flat');
floorVertexCount = size(vFloor,1);
updatedFloorPos = [floorPose * [vFloor,ones(floorVertexCount,1)]']';
floor_h.Vertices = updatedFloorPos(:,1:3);
drawnow();
hold on

%% Load Robot
[f,v,data] = plyread('iRobot.ply','tri');
VertexCount = size(v,1);
VertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
MidPoint = sum(v)/VertexCount;
Verts= v - repmat(MidPoint,VertexCount,1);
%Plot the robot
robot_h = trisurf(f,v(:,1),v(:,2),v(:,3),'FaceVertexCData',VertexColours,'EdgeColor','interp','EdgeLighting','flat');
%Assigning position
updatedPosition = [robotPosition * [Verts,ones(VertexCount,1)]']';
%Update the vertices
robot_h.Vertices = updatedPosition(:,1:3);
axis auto equal

%%Initialize communnicationusing global value in a function MATLAB
arduinoObj = serialport(SerialPort,BaudRate);
configureTerminator(arduinoObj,"CR/LF");
flush(arduinoObj);

while 1
    %% how to communicate with arduinno ?
    buffer = arduinoObj.readline;
    
    switch buffer
        case "UP"
            CMD_MOVE(0.05);
        case "DOWN"
            CMD_MOVE(-0.05);
        case "RIGHT"
            CMD_ROTATE(1,5);
        case "LEFT"
            CMD_ROTATE(0,5);
    end
    
    updatedPosition = [robotPosition*[v,ones(size(v,1),1)]']';
    robot_h.Vertices = updatedPosition(:,1:3);
    drawnow()
    
    %% read input
    %% execute
    
end


%% Move Robot
function  CMD_MOVE(distance)
global robotPosition
robotPosition = robotPosition * transl(distance,0,0);
end

%% Rotate Robot
%direction: 0 is CCW, 1 is CCW
function CMD_ROTATE(direction,angle)
if direction == 1
    angle = -angle;
end
global robotPosition
robotPosition = robotPosition * trotz(angle,'deg');
end