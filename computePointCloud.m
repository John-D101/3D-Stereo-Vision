clear variables; close all; clc;

%% Generate the circle pattern world points
numSide = 2; %Number of dots on the single side row of the pattern
numTop = 4; %Number of dots in first 2 rows of pattern
centreDistance = 5; %Distance between circles on the same row in mm
patternDims = [numSide, numTop];

worldPoints = generateCircleGridPoints(patternDims, centreDistance);

%% Detect the calibration pattern from the calibration image set
calibImageDir = "Calibration Set";
calibLeftImages = imageDatastore(fullfile(calibImageDir,"left\"));
calibRightImages = imageDatastore(fullfile(calibImageDir,"right\"));
[imagePoints, imagesUsed] = detectCircleGridPoints(calibLeftImages.Files, calibRightImages.Files, patternDims);

%% Calibrate the camera parameters and rectify the dataset
stereoCamPara = estimateCameraParameters(imagePoints, worldPoints, ImageSize=[768,1024]);
sampleImageDir = "Run _\";
sampleLeftImages = imageDatastore(fullfile(sampleImageDir,"raw\left\"));
sampleRightImages = imageDatastore(fullfile(sampleImageDir,"raw\right\"));

for image_num = 1:size(sampleLeftImages.Files,1)
    [J1,J2,ReProj] = rectifyStereoImages(sampleLeftImages,sampleRightImages, stereoCamPara);
    imwrite(J1, fullfile(sampleImageDir,"rectified\left\",sampleLeftImages.Files{image_num}(end-20:end)));
    imwrite(J2, fullfile(sampleImageDir,"rectified\right\",sampleRightImages.Files{image_num}(end-21:end)));
    disparityMap = disparitySGM(J1,J2);
    xyzPoints = reconstructScene(J1,ReProj);
    ptCloudNoise = pointCloud(xyzPoints./1000, 'Color', images(:,:,1:3));
    ptCloudRed = pcdenoise(ptCloudNoise, NumNeighbors=30);
end

player3D = pcplayer([-3, 3], [-3, 3], [0, 6], 'VerticalAxis', 'y', ...
    'VerticalAxisDir', 'down');
view(player3D, ptCloudRed);