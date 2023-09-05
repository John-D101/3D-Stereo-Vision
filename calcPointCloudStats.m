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

%% Rectify the original dataset and store the dataset together in ptCloudGrouped
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

%% Fit a plane to each of the point clouds
%fittedPlanemodel stores the plane model inc. eq. on first row and
%histogram of distances on second row
fittedPlaneModel = cell(2,size(ptCloudGrouped,2));

for pc_num = 1:size(ptCloudGrouped,2)
    [model,inlierIndices,outlierIndices,meanError] = pcfitplane(ptCloudGrouped{pc_num},0.01);
    fittedPlaneModel(1,pc_num) = [model,meanError];
end

%% Calculate the distance between each point and the fitted plane
for pc_num = 1:size(ptCloudGrouped,2)
    %Preallocation and storing the normal vector
    distArray = zeros(1,size(ptCloudGrouped{pc_num}.Location,1));
    normVec = fittedPlaneModel(1,pc_num).Normal;
    ptVec = zeros(1,3);

    for pt_num = 1:size(ptCloudGrouped{pc_num}.Location,2)
        ptsInPtCloud = ptCloudGrouped{pc_num}.Location; %Retrieve the matrix of all points in the cloud
        ptVec = ptsInPtCloud(pt_num,:); %Retrieve the individual point
        %If a NaN is any of the values, its unusable
        if isNaN(ptsInPtCloud(pt_num,1)) || isNaN(ptsInPtCloud(pt_num,2)) || isNaN(ptsInPtCloud(pt_num,3))
            continue
        end
        %Vector projection where a is the point, b is the normal vector and
        %we want a in the direction of b, i.e. distance
        distArray(1,pt_num) = norm(dot(ptVec,normVec)/dot(normVec,normVec)*normVec);
    end
    fittedPlaneModel(2,pc_num) = histogram(distArray);
end
