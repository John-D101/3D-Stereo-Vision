clear variables; close all; clc; 

%% Read all input files and initialise the reprojection matrix
[images, depth] = read_im_from_run("Run 2", 6);

%Create the reprojection matrix from the individual projection matrices
%https://stackoverflow.com/questions/30607715/reprojection-matrix-q-from-projection-matrices-p-left-and-p-right
ProjLeft = [872.76 0 766.88 0; 0 872.76 574.20 0; 0 0 1 0];
ProjRight = [872.76 0 766.88 -43.647; 0 872.76 574.20 0; 0 0 1 0];
ReProj = reproj_mat(ProjLeft, ProjRight);

%Read the disparity map in and adjust the invalid values to zero
disparityRaw = depth(:,:,1);
depth(logical((round(depth(:,:,1),4)==0.5020) .* (round(depth(:,:,2),4)==0.5020) .* (round(depth(:,:,3),4)==0.5020))) = NaN;

%Display all three disparity maps to verify it worked
subplot(1,2,1)
imshow(disparityRaw)
subplot(1,2,2)
imshow(depth(:,:,1:3))

%% Compute the point cloud for the system
xyzPoints = reconstructScene(im2gray(depth(:,:,1:3)),ReProj);
ptCloudNoise = pointCloud(xyzPoints./1000.*16, 'Color', images(:,:,1:3));
ptCloudRed = pcdenoise(ptCloudNoise, NumNeighbors=30);

%% Display the point cloud
player3D = pcplayer([-3, 3], [-3, 3], [0, 6], 'VerticalAxis', 'y', ...
    'VerticalAxisDir', 'down');
view(player3D, ptCloudRed);