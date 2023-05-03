clear variables; close all; clc;
%% Calibrate the camera parameters
%Generate the world points for the circle grid
numSide = 2; %Number of dots on the single side row of the pattern
numTop = 4; %Number of dots in first 2 rows of pattern
centreDistance = 5; %Distance between circles on the same row in mm
patternDims = [numSide, numTop];
worldPoints = generateCircleGridPoints(patternDims, centreDistance);

% Detect the calibration pattern from the calibration image set
calibImageDir = "Calibration Set";
calibLeftImages = imageDatastore(fullfile(calibImageDir,"left\"));
calibRightImages = imageDatastore(fullfile(calibImageDir,"right\"));
[imagePoints, imagesUsed] = detectCircleGridPoints(calibLeftImages.Files, calibRightImages.Files, patternDims);
stereoCamPara = estimateCameraParameters(imagePoints, worldPoints, ImageSize=[768,1024]);

%% Rectify the original dataset and save a copy of it then make a point cloud
%Read the images
sampleImageDir = "Run _\";
sampleLeftImages = imageDatastore(fullfile(sampleImageDir,"raw\left\"));
sampleRightImages = imageDatastore(fullfile(sampleImageDir,"raw\right\"));

for image_num = 1:size(sampleLeftImages.Files,1)
    [J1,J2,ReProj] = rectifyStereoImages(sampleLeftImages,sampleRightImages, stereoCamPara);
    imwrite(J1, fullfile(sampleImageDir,"rectified\left\", sampleLeftImages.Files{image_num}(end-20:end)));
    imwrite(J2, fullfile(sampleImageDir,"rectified\right\", sampleRightImages.Files{image_num}(end-21:end)));
    disparityMap = disparitySGM(J1,J2);
    xyzPoints = reconstructScene(J1,ReProj);
    ptCloudNoise = pointCloud(xyzPoints./1000, 'Color', images(:,:,1:3));
    ptCloudRed = pcdenoise(ptCloudNoise, NumNeighbors=4);
    pcwrite(ptCloudRed, fullfile(sampleImageDir,"pcloud\", sampleRightImages.Files{image_num}(end-21:end-4),".ply"))
end

%% Join the point clouds together
% Initial parameters
gridSize = 0.1;
mergeSize = 0.015;

%Read the initial point cloud
ptCloudRef = pcread(fullfile(sampleImageDir,"pcloud\", sampleRightImages.Files{1}(end-21:end-4),".ply"));
ptCloudCurrent = pcread(fullfile(sampleImageDir,"pcloud\", sampleRightImages.Files{2}(end-21:end-4),".ply"));

%Downsample the points to improve accuracy and speed
fixed = pcdownsample(ptCloudRef, 'gridAverage',gridSize);
moving = pcdownsample(ptCloudCurrent, 'gridAverage',gridSize);

%Find the transform for the point cloud and adjust the second cloud to
%match the first
tform = pcregistericp(moving,fixed,'Metric','pointToPlane','Extrapolate',true);
ptCloudAligned = pctransform(ptCloudCurrent,tform);
ptCloudScene = pcmerge(ptCloudRef,ptCloudAligned,mergeSize);

%Loop over subsequent clouds, moving our reference cloud along
for cloud_num = 3:size(sampleLeftImages.Files,1)
    ptCloudCurrent = pcread(fullfile(sampleImageDir,"pcloud\", sampleRightImages.Files{cloud_num}(end-21:end-4),".ply"));
    fixed = moving;
    moving = pcdownsample(ptCloudCurrent, 'gridAverage',gridSize);

    %Accumulate the transform from the first point cloud
    tform = pcregistericp(moving,fixed,'Metric','pointToPlane','Extrapolate',true);
    accumTform = rigidtform3d(accumTform.A * tform.A);
    ptCloudAligned = pctransform(ptCloudCurrent, accumTform);
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergesize);
end

hAxes = pcshow(ptCloudScene,'VerticalAxis','Y','VerticalAxisDir','Down');
title('Updated world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

