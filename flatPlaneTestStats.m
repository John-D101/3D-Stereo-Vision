clear variables; close all; clc;
%% Calibrate the camera parameters
%Generate the world points for the circle grid
numSide = 4; %Number of dots on the single side row of the pattern
numTop = 11; %Number of dots in first 2 rows of pattern
centreDistance = 20; %Distance between circles on the same row in mm
patternDims = [numSide, numTop];
worldPoints = generateCircleGridPoints(patternDims, centreDistance);

% Detect the calibration pattern from the calibration image set
calibImageDir = "FYP Testing Set 2/calibration";
calibLeftImages = imageDatastore(fullfile(calibImageDir,"\left\"));
calibRightImages = imageDatastore(fullfile(calibImageDir,"\right\"));
[imagePoints, imagesUsed] = detectCircleGridPoints(calibLeftImages.Files, calibRightImages.Files, patternDims);
stereoCamPara = estimateCameraParameters(imagePoints, worldPoints, ImageSize=[768,1024]);

%Save the camera parameters
save("CameraParameters.mat","stereoCamPara","-mat")

%% Import camera parameters
load("CameraParameters.mat")

%% Rectify the original dataset and store the dataset together in ptCloudGrouped
%Read the images
sampleImageDir = "FYP Testing Set 2";
sampleLeftImages = imageDatastore(fullfile(sampleImageDir,"flat plane test\left\"));
sampleRightImages = imageDatastore(fullfile(sampleImageDir,"flat plane test\right\"));
ptCloudGrouped = cell(1,size(sampleLeftImages.Files,1));

for image_num = 1:size(sampleLeftImages.Files,1)
    [J1,J2,ReProj] = rectifyStereoImages(imread(sampleLeftImages.Files{image_num}),imread(sampleRightImages.Files{image_num}), stereoCamPara);
    %imwrite(J1, fullfile(sampleImageDir,"/rectified/left/", sampleLeftImages.Files{image_num}(end-20:end)));
    %imwrite(J2, fullfile(sampleImageDir,"/rectified/right/", sampleRightImages.Files{image_num}(end-21:end)));
    disparityMap = disparitySGM(rgb2gray(J1),rgb2gray(J2));
    xyzPoints = reconstructScene(disparityMap,ReProj);
    ptCloudNoise = pointCloud(xyzPoints./1000,"Color",J1);

    % hAxes = pcshow(ptCloudNoise,'VerticalAxis','Y','VerticalAxisDir','Down');
    % title('Current world')
    % xlabel('X (m)')
    % ylabel('Y (m)')
    % zlabel('Z (m)')
    % ptRegion = input("What region is the point cloud?");
    ptCloudRed = select(ptCloudNoise, findPointsInROI(ptCloudNoise, [-0.2 0.2 -0.2 0.2 -0.2 0.3]));

    ptCloudRed = pcdenoise(ptCloudRed, NumNeighbors=10);
    ptCloudGrouped{image_num} = ptCloudRed;
    %pcwrite(ptCloudRed, fullfile(sampleImageDir,"/pcloud", strcat(sampleRightImages.Files{image_num}(end-21:end-4),".ply")))
end

%% Fit a plane to each of the point clouds
%fittedPlanemodel stores the plane model inc. eq. on first row and
%histogram of distances on second row
fittedPlaneModel = cell(2,size(ptCloudGrouped,2)*2);
ptCloudTrimmed = cell(size(ptCloudGrouped,1),size(ptCloudGrouped,2));

for pc_num = 1:2:size(ptCloudGrouped,2)*2
    %Include all points that are within 2cm of the proposed plane
    [model,inlierIndices,outlierIndices,meanError] = pcfitplane(ptCloudGrouped{(pc_num+1)/2}, 0.02);
    ptCloudTrimmed{(pc_num+1)/2} = select(ptCloudGrouped{(pc_num+1)/2}, inlierIndices);
    fittedPlaneModel(1,pc_num) = {model};
    fittedPlaneModel(1,pc_num+1) = {meanError};
end

%% Calculate the distance between each point and the fitted plane
%Setting grid text
gridText = ["5cm", "5cm", "5cm", ...
    "4cm", "4cm", "4cm", ...
    "3cm", "3cm", "3cm", ...
    "2cm", "2cm", "2cm", ...
    "1.8cm", "1.8cm", "1.8cm", ...
    "1.6cm", "1.6cm", "1.6cm", ...
    "1.4cm", "1.4cm", "1.4cm", ...
    "1.2cm", "1.2cm", "1.2cm", "1.2cm", "1.2cm",...
    "1cm", "1cm", "1cm"];
statsVec = zeros(3,size(ptCloudTrimmed,2));

for pc_num = 1:2:size(ptCloudTrimmed,2)*2
    %Storing the normal vector
    distArray = zeros(1,size(ptCloudTrimmed{(pc_num+1)/2}.Location,1));
    normVec = fittedPlaneModel{1,pc_num}.Normal;
    ptVec = zeros(1,3);
    ptsInPtCloud = ptCloudTrimmed{(pc_num+1)/2}.Location; %Retrieve the matrix of all points in the cloud
    for pt_num = 1:size(ptsInPtCloud,1)
        ptVec = ptsInPtCloud(pt_num,:); %Retrieve the individual point
        %If a NaN is any of the values, its unusable
        if ~(isnan(ptsInPtCloud(pt_num,1)) || isnan(ptsInPtCloud(pt_num,2)) || isnan(ptsInPtCloud(pt_num,3)))
            %Vector projection where a is the point, b is the normal vector and
            %we want a in the direction of b, i.e. distance
            distArray(1,pt_num) = norm(dot(ptVec,normVec)/dot(normVec,normVec)*normVec);
        end
    end
    %Centre the distances
    distArray = distArray - mean(distArray);

    %Visualisation of curves
    fittedPlaneModel(2,pc_num) = {histfit(distArray,[],"logistic")};
    xlabel("Perpendicular distance from plane (m)")
    ylabel("Frequency of distance")
    title(sprintf("Grid = %s",gridText((pc_num+1)/2)))
    %exportgraphics(fittedPlaneModel{2,pc_num}, sprintf("BestFitPointCloud%s.png",pc_num))
    statsVec(1,(pc_num+1)/2) = std(distArray); 
    statsVec(2,(pc_num+1)/2) = skewness(distArray);
    statsVec(3,(pc_num+1)/2) = kurtosis(distArray);
   
    %fittedPlaneModel(2,pc_num) = {fitdist(distArray',"normal")};
    %fittedPlaneModel(2,pc_num) = {histogram(distArray)};
    % logistic distribution might be useful
end

%Save the calculated distribution parameters
save("PT Result Parameters.mat", "statsVec")