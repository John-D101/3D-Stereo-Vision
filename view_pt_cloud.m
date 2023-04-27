clear variables; close all; clc; 

%% Read all input files and initialise the reprojection matrix
disparityRaw = imread("depth\image000001_disp_raw.png"); 
frameLeftRect = imread("images\image000001_left.png");
%frameRightRect = imread("images\image00000#_right.png");
%reprojectionMatrix = [1 0 0 -cx; 0 1 0 -cy; 0 0 0 f; 0 0 1/b 0];

% Read the disparity map in and adjust the invalid values to zero

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
%xyzPoints = reconstructScene(disparityDouble,reprojectionMatrix);
%ptCloud = pointCloud(xyzPoints./1000, 'Color', frameLeftRect)


%% Display the point cloud
%player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y', ...
%    'VerticalAxisDir', 'down');
%view(player3D, ptCloud);