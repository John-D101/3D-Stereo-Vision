clear variables; close all; clc;
%% Calibrate the camera parameters
%Generate the world points for the circle grid
numSide = 4; %Number of dots on the single side row of the pattern
numTop = 11; %Number of dots in first 2 rows of pattern
centreDistance = 20; %Distance between circles on the same row in mm
patternDims = [numSide, numTop];
worldPoints = generateCircleGridPoints(patternDims, centreDistance);

% Detect the calibration pattern from the calibration image set
calibImageDir = "Calibration Set";
calibLeftImages = imageDatastore(fullfile(calibImageDir,"/left/"));
calibRightImages = imageDatastore(fullfile(calibImageDir,"/right/"));
[imagePoints, imagesUsed] = detectCircleGridPoints(calibLeftImages.Files, calibRightImages.Files, patternDims);
stereoCamPara = estimateCameraParameters(imagePoints, worldPoints, ImageSize=[768,1024]);

%% Rectify the original dataset and save a copy of it then make a point cloud
%Read the images
sampleImageDir = "Run 4";
sampleLeftImages = imageDatastore(fullfile(sampleImageDir,"/raw/left/"));
sampleRightImages = imageDatastore(fullfile(sampleImageDir,"/raw/right/"));
ptCloudGrouped = cell(1,size(sampleLeftImages.Files,1));

for image_num = 1:size(sampleLeftImages.Files,1)
    [J1,J2,ReProj] = rectifyStereoImages(imread(sampleLeftImages.Files{image_num}),imread(sampleRightImages.Files{image_num}), stereoCamPara);
    imwrite(J1, fullfile(sampleImageDir,"/rectified/left/", sampleLeftImages.Files{image_num}(end-20:end)));
    imwrite(J2, fullfile(sampleImageDir,"/rectified/right/", sampleRightImages.Files{image_num}(end-21:end)));
    disparityMap = disparitySGM(rgb2gray(J1),rgb2gray(J2));
    xyzPoints = reconstructScene(disparityMap,ReProj);
    ptCloudNoise = pointCloud(xyzPoints./1000,"Color",J1);
    ptCloudRed = select(ptCloudNoise, findPointsInROI(ptCloudNoise, [-0.5 0.5 -0.1 0.5 -0.5 0.5]));
    ptCloudRed = pcdenoise(ptCloudRed, NumNeighbors=10);
    ptCloudGrouped{image_num} = ptCloudRed;
    %pcwrite(ptCloudRed, fullfile(sampleImageDir,"/pcloud", strcat(sampleRightImages.Files{image_num}(end-21:end-4),".ply")))
end

%% Join the point clouds together
% Initial parameters
gridSize = 0.01;
mergeSize = 0.0005;

%Read the initial point cloud
%ptCloudRef = pcread(fullfile(sampleImageDir,"/pcloud", strcat(sampleRightImages.Files{1}(end-21:end-4),".ply")));
%ptCloudCurrent = pcread(fullfile(sampleImageDir,"/pcloud", strcat(sampleRightImages.Files{2}(end-21:end-4),".ply")));
ptCloudRef = ptCloudGrouped{1};
ptCloudCurrent = ptCloudGrouped{2};

%Downsample the points to improve accuracy and speed
fixed = pcdownsample(ptCloudRef, 'gridAverage',gridSize);
moving = pcdownsample(ptCloudCurrent, 'gridAverage',gridSize);

%Find the transform for the point cloud and adjust the second cloud to
%match the first
tform = pcregistericp(moving,fixed,'Metric','pointToPlane','Extrapolate',true);
accumTform = tform; 
ptCloudAligned = pctransform(ptCloudCurrent,tform);
ptCloudScene = pcmerge(ptCloudRef,ptCloudAligned,mergeSize);

%Loop over subsequent clouds, moving our reference cloud along
for cloud_num = 3:size(sampleLeftImages.Files,1)
    %ptCloudCurrent = pcread(fullfile(sampleImageDir,"pcloud/", strcat(sampleRightImages.Files{cloud_num}(end-21:end-4),".ply")));
    ptCloudCurrent = ptCloudGrouped{cloud_num};
    fixed = moving;
    moving = pcdownsample(ptCloudCurrent, 'gridAverage',gridSize);

    %Accumulate the transform from the first point cloud
    tform = pcregistericp(moving,fixed,'Metric','pointToPlane','Extrapolate',true);
    accumTform = rigidtform3d(accumTform.A * tform.A);
    ptCloudAligned = pctransform(ptCloudCurrent, accumTform);
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
end
%Save the combined point cloud
pcwrite(ptCloudScene, fullfile(sampleImageDir,"/pcloud", strcat(sampleImageDir,".ply")))

hAxes = pcshow(ptCloudScene,'VerticalAxis','Y','VerticalAxisDir','Down');
title('Updated world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

