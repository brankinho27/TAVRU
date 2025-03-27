clear all; close all;

%% Import the video file
obj = VideoReader('Videos/cam_front.avi');
vid = read(obj);
frames = obj.NumberOfFrames;

%% Get the frames and save the images (every 5 frames)
cd frames_front

for i = 1 : 5 : frames
    % Image name
    number = sprintf('%04d', i);
    fileName = strcat(number, '.jpg');
    
    % Exporting the frames
    Image = vid(:, :, :, i);
    imwrite(Image, fileName);
end

cd ..
