
%% Load floor 
floorPose=transl(0,0,0);
[fFloor,vFloor,dataFloor] = plyread('floor.ply','tri');
floorVertexColours = [dataFloor.vertex.red,dataFloor.vertex.green,dataFloor.vertex.blue] / 255;
hold on;       
floor_h = trisurf(fFloor,vFloor(:,1),vFloor(:,2),vFloor(:,3),'FaceVertexCData',floorVertexColours,'EdgeColor','interp','EdgeLighting','flat');
floorVertexCount = size(vFloor,1);
updatedFloorPos = [floorPose * [vFloor,ones(floorVertexCount,1)]']';
floor_h.Vertices = updatedFloorPos(:,1:3);
drawnow();
hold on

%% Load Robot
robotPosition = transl(0,0,0.1);  % robot initial position is (0,0)
robot_heading = 0;                % original robot heading is x-axis(theta = 0)
[f,v,data] = plyread('iRobot.ply','tri');
VertexCount = size(v,1);
VertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;   
MidPoint = sum(v)/VertexCount;
Verts= v - repmat(MidPoint,VertexCount,1);
%Plot the robot
robot_h = trisurf(f,v(:,1),v(:,2),v(:,3),'FaceVertexCData',VertexColours,'EdgeColor','interp','EdgeLighting','flat');
%Assigning position
updatedBottom = [robotPosition * [Verts,ones(VertexCount,1)]']';
%Update the vertices
robot_h.Vertices = updatedBottom(:,1:3);
axis auto equal




%% Move Robot 
function Tmatrix = CMD_MOVE(distance)
Tmatrix = transl(distance,0,0)

end

%% Rotate Robot
%angle value POSITIVE is CCW, NEGATIVE is CCW
function Tmatrix = CMD_ROTATE(angle)
Tmatrix = trotz()
end