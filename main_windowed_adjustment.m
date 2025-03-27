clear all; close all;

%% Open frames and create datastore
dataFolder = fullfile('frames_front'); 
images = imageDatastore(dataFolder);

% Create an empty imageviewset object to manage the data associated with each view
vSet = imageviewset;

% Create the camera intrinsics object
R = [-0.338651, 0.142042, -0.030265];
T = [-0.000232, 0.000173];
K = [1110.344126, -0.598042, 1039.193868; 
    0.000000, 1107.604563, 796.997243; 
    0.000000, 0.000000, 1.000000];

intrinsics = cameraParameters('RadialDistortion', R, 'TangentialDistortion', T, 'IntrinsicMatrix', K');

%% Read and display the first image
Irgb = readimage(images, 1);
player = vision.VideoPlayer;
step(player, Irgb);

% Convert to gray scale and undistort
prevI = undistortImage(im2gray(Irgb), intrinsics); 

% Detect and extract SURF features 
prevPoints = detectSURFFeatures(prevI, 'MetricThreshold', 200);
numPoints = 500;
prevPoints = selectUniform(prevPoints, numPoints, size(prevI));
prevFeatures = extractFeatures(prevI, prevPoints, 'Upright', true);

% Add the first view. Place the camera associated with the first view
% at the origin, oriented along the Z-axis
viewId = 1;
vSet = addView(vSet, viewId, rigid3d(eye(3), [0 0 0]), 'Points', prevPoints);

% Create the 3D plot
view(gca, 3);
set(gca, 'CameraUpVector', [0, 0, -1]);
grid on
xlabel('x'); ylabel('y'); zlabel('z');
hold on

% Plot estimated camera pose
cameraSize = 1;
camPose = poses(vSet);
camEstimated = plotCamera(camPose, 'Size', cameraSize, 'Color', 'k', 'Opacity', 0);

% Initialize camera trajectories
trajectoryEstimated = plot3(0, 0, 0, 'b-');
legend('Estimated Trajectory');
title('Camera Trajectory');


%% Read and display the second image
viewId = 2;
Irgb = readimage(images, viewId);
step(player, Irgb);

% Convert to gray scale and undistort
I = undistortImage(im2gray(Irgb), intrinsics);

% Match features between the previous and the current image (first and second)
[currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(prevFeatures, I);

% Estimate the pose of the current view relative to the previous view
[orient, loc, inlierIdx] = helperEstimateRelativePose(...
    prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), intrinsics);

% Exclude epipolar outliers
indexPairs = indexPairs(inlierIdx, :);
    
% Add the current view to the view set
vSet = addView(vSet, viewId, rigid3d(orient, loc), 'Points', currPoints);

% Store the point matches between the previous and the current views
vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);

% Update camera trajectory plot
helperUpdateCameraPlots(viewId, camEstimated, poses(vSet));
helperUpdateCameraTrajectories(viewId, trajectoryEstimated, poses(vSet));

prevI = I;
prevFeatures = currFeatures;
prevPoints   = currPoints;


%% Next 30 frames (global adjustment - all previous views) 
for viewId = 3:30
    % Read and display the next image
    Irgb = readimage(images, viewId);
    step(player, Irgb);
    
    % Convert to gray scale and undistort
    I = undistortImage(im2gray(Irgb), intrinsics);
    
    % Match points between the previous and the current image
    [currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(prevFeatures, I);
    
    % Eliminate outliers from feature matches
    inlierIdx = helperFindEpipolarInliers(prevPoints(indexPairs(:,1)),...
        currPoints(indexPairs(:, 2)), intrinsics);
    indexPairs = indexPairs(inlierIdx, :);
    
    % Triangulate points from the previous two views, and find the 
    % corresponding points in the current view
    [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(vSet,...
        intrinsics, indexPairs, currPoints);
        
    % Estimate the world camera pose for the current view
    [orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, intrinsics);
    
    % Add the current view to the view set
    vSet = addView(vSet, viewId, rigid3d(orient, loc), 'Points', currPoints);
    
    % Store the point matches between the previous and the current views
    vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);    
    
    % Find point tracks spanning multiple views
    tracks = findTracks(vSet); 
        
    % Get camera poses for all views
    camPoses = poses(vSet);
    
    % Triangulate initial locations for the 3-D world points
    xyzPoints = triangulateMultiview(tracks, camPoses, intrinsics);
    
    % Refine camera poses using global bundle adjustment
    [~, camPoses] = bundleAdjustment(xyzPoints, tracks, camPoses, ...
        intrinsics, 'PointsUndistorted', true, 'AbsoluteTolerance', 1e-12,...
        'RelativeTolerance', 1e-12, 'MaxIterations', 200, 'FixedViewID', 1);
        
    % Update view set
    vSet = updateView(vSet, camPoses);
    
    % Update camera trajectory plot
    helperUpdateCameraPlots(viewId, camEstimated, poses(vSet));
    helperUpdateCameraTrajectories(viewId, trajectoryEstimated, poses(vSet));
    
    prevI = I;
    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
end


%% Rest of the images (windowed ajudstment - optimizes only last 15 views)
for viewId = 31:numel(images.Files)
    % Read and display the next image
    Irgb = readimage(images, viewId);
    step(player, Irgb);
    
    % Convert to gray scale and undistort
    I = undistortImage(im2gray(Irgb), intrinsics);

    % Match points between the previous and the current image
    [currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(prevFeatures, I);    
          
    % Triangulate points from the previous two views, and find the 
    % corresponding points in the current view
    [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(vSet, ...
        intrinsics, indexPairs, currPoints);
    
    % Estimate the world camera pose for the current view
    [orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, intrinsics);
    
    % Add the current view and connection to the view set
    vSet = addView(vSet, viewId, rigid3d(orient, loc), 'Points', currPoints);
    
    % Store the point matches between the previous and the current views
    vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);
        
    % Refine estimated camera poses using windowed bundle adjustment 
    % Run the optimization every 5th view
    if mod(viewId, 5) == 0        
        % Find point tracks in the last 15 views and triangulate
        windowSize = 15;
        startFrame = max(1, viewId - windowSize);
        tracks = findTracks(vSet, startFrame:viewId);
        camPoses = poses(vSet, startFrame:viewId);
        [xyzPoints, reprojErrors] = triangulateMultiview(tracks, camPoses, intrinsics);
                                
        % Hold the first two poses fixed, to keep the same scale
        fixedIds = [startFrame, startFrame+1];
        
        % Exclude points and tracks with high reprojection errors
        idx = reprojErrors < 2;
        
        % Refine camera poses
        [~, camPoses] = bundleAdjustment(xyzPoints(idx, :), tracks(idx), ...
            camPoses, intrinsics, 'FixedViewIDs', fixedIds, ...
            'PointsUndistorted', true, 'AbsoluteTolerance', 1e-12,...
            'RelativeTolerance', 1e-12, 'MaxIterations', 200);
        
        % Update view set
        vSet = updateView(vSet, camPoses);
    end
    
    % Update camera trajectory plot
    helperUpdateCameraPlots(viewId, camEstimated, poses(vSet));
    helperUpdateCameraTrajectories(viewId, trajectoryEstimated, poses(vSet));
    
    prevI = I;
    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
end