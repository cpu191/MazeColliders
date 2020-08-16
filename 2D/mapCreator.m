% clear  % Map without white background will not work
% image = imread('box.png');
% grayimage = rgb2gray(image);
% bwimage = grayimage < 0.5;
% grid = binaryOccupancyMap(bwimage,10)
% show(grid)

map = binaryOccupancyMap(10,10,10);
x =1:10;
y(1:10) = 0;
points = [x' y]
setOccupancy(map, points, ones(10,1))
inflate(map, 0.5)
show(map)


% x = [1.2; 2.3; 3.4; 4.5; 5.6];
% y = [5.0; 4.0; 3.0; 2.0; 1.0];
% 
% setOccupancy(map, [x y], ones(5,1))