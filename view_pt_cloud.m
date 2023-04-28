clear variables; close all; clc; 

%% Read all input files and initialise the reprojection matrix
disparityRaw = imread("Run 2\depth\image000000_disp_raw.png"); 
frameLeftRect = imread("Run 2\images\image000000_left.png");
%frameRightRect = imread("Run 2\images\image00000#_right.png");


%Create the reprojection matrix from the individual projection matrices
%https://stackoverflow.com/questions/30607715/reprojection-matrix-q-from-projection-matrices-p-left-and-p-right
ProjLeft = [872.76 0 766.88 0; 0 872.76 574.20 0; 0 0 1 0];
ProjRight = [872.76 0 766.88 -43.647; 0 872.76 574.20 0; 0 0 1 0];
%reprojectionMatrix = [1 0 0 -cx; 0 1 0 -cy; 0 0 0 f; 0 0 1/b 0];
ReProj = [1 0 0 -ProjLeft(1,3);0 1 0 -ProjLeft(2,3); 0 0 0 ProjLeft(1,1); 0 0 -ProjRight(1,1)/ProjRight(1,4) (ProjLeft(1,3)-ProjRight(1,3))*ProjRight(1,1)/ProjRight(1,4)];

%Read the disparity map in and adjust the invalid values to zero
disparityNan = disparityRaw;
disparityNan(disparityNan==65520) = NaN;
disparityDouble = im2double(disparityNan);

%Display all three disparity maps to verify it worked
subplot(1,3,1)
imshow(disparityRaw)
subplot(1,3,2)
imshow(disparityNan)
subplot(1,3,3)
imshow(disparityDouble)

%% Compute the point cloud for the system
xyzPoints = reconstructScene(disparityDouble,ReProj);
ptCloud = pointCloud(xyzPoints./1000, 'Color', frameLeftRect);

%% Display the point cloud
player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y', ...
    'VerticalAxisDir', 'down');
view(player3D, ptCloud);