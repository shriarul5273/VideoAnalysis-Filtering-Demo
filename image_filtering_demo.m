clf;
clear;
close all;
clc;

%% input image
% Assuming using default matlab images, 
% if not provide full path of the image
rgbImage = imread('yellowlily.jpg'); 
% rgbImage = imread('cameraman.tif'); 


%% Adding noise to  the image 
noisedImage = imnoise(rgbImage,"gaussian",0.05); 
% noisedImage = imnoise(rgbImage, "salt & pepper",0.02);


%% Filteting
% Apply the filter to each channel of the RGB image

%%% Avearge filter 
% F = fspecial('average',[7,7]);
% denoisedImage = imfilter(noisedImage, F); 

%%% Bilateral filter
% denoisedImage = imbilatfilt(noisedImage);

%%% Guided Image Filtering
denoisedImage = imguidedfilter(noisedImage);

%%% Median filtering
% denoisedImage =  medfilt3(noisedImage,[[3,3] 1]);

%%% Sobel operation
% sobelXKernel = [-1 0 1; -2 0 2; -1 0 1];
% denoisedImage = imfilter(noisedImage, sobelXKernel); 

% sobelYKernel = [-1 -2 -1; 0 0 0; 1 2 1];
% denoisedImage = imfilter(noisedImage, sobelYKernel); 



%% GUI for visualization.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Figure/parent container (uifigure) properties
App = uifigure('Scrollable','on','Name','Image Zoom','NumberTitle','off');
App_Width = 1200; App_Height = 800;
App.Position = [0 0 App_Width App_Height];

% Slider label (uilabel) properties
Slider_Label = uilabel('Parent',App);
Slider_Label.Text = "Zoom Factor";
Slider_Label.Position = [25 20 200 100];

% Slider (uislider) properties
Slider = uislider('Parent',App);
Slider.Limits = [1 10];
Slider.Value = 1;
Slider_Width = App_Width - 50;
Margin = (App_Width - Slider_Width) / 2;
Slider.Position = [Margin 50 Slider_Width 3];
Slider.MajorTicks = 1:1:10;
Slider.FontSize = 12;
Red = 87; Green = 207; Blue = 220;
Slider.FontColor = [Red/255 Green/255 Blue/255];

% Image display axes properties for original, noised, and denoised images
originalImageAxes = uiaxes('Parent',App);
originalImageAxes.Position = [50 450 400 300];
imshow(rgbImage, 'Parent', originalImageAxes);
title(originalImageAxes, 'Original Image');

noisedImageAxes = uiaxes('Parent',App);
noisedImageAxes.Position = [550 450 400 300];
imshow(noisedImage, 'Parent', noisedImageAxes);
title(noisedImageAxes, 'Noised Image');

denoisedImageAxes = uiaxes('Parent',App);
denoisedImageAxes.Position = [1050 450 400 300];
imshow(denoisedImage, 'Parent', denoisedImageAxes);
title(denoisedImageAxes, 'Filtered Image');

% Axes for displaying cropped images
croppedImageAxes = uiaxes('Parent',App);
croppedImageAxes.Position = [50 100 400 300];
title(croppedImageAxes, 'Cropped Region');
axis(croppedImageAxes, 'off');

croppedNoisedImageAxes = uiaxes('Parent',App);
croppedNoisedImageAxes.Position = [550 100 400 300];
title(croppedNoisedImageAxes, 'Noised Region');
axis(croppedNoisedImageAxes, 'off');

croppedDenoisedImageAxes = uiaxes('Parent',App);
croppedDenoisedImageAxes.Position = [1050 100 400 300];
title(croppedDenoisedImageAxes, 'Filtered Region');
axis(croppedDenoisedImageAxes, 'off');

% Initialize the handles for the rectangles
rectangleHandle = [];
rectangleHandleNoise = [];
rectangleHandleDenoise = [];

% Infinite loop for continuous ROI selection
while ishandle(App)
    % Draw the rectangle and get its position
    roi = drawrectangle(originalImageAxes);
    pos = roi.Position;
    pos = round(pos);

    % Ensure the position values are within valid limits
    pos(1) = max(1, pos(1));
    pos(2) = max(1, pos(2));
    pos(3) = min(size(rgbImage, 2) - pos(1) + 1, pos(3));
    pos(4) = min(size(rgbImage, 1) - pos(2) + 1, pos(4));

    % Extract the cropped regions, including all three color channels
    croppedImage = rgbImage(pos(2):pos(2)+pos(4)-1, pos(1):pos(1)+pos(3)-1, :);
    croppedNoisedImage = noisedImage(pos(2):pos(2)+pos(4)-1, pos(1):pos(1)+pos(3)-1, :);
    croppedDenoisedImage = denoisedImage(pos(2):pos(2)+pos(4)-1, pos(1):pos(1)+pos(3)-1, :);

   % Display the cropped images 
    imshow(croppedImage, 'Parent', croppedImageAxes);
    imshow(croppedNoisedImage, 'Parent', croppedNoisedImageAxes);
    imshow(croppedDenoisedImage, 'Parent', croppedDenoisedImageAxes);


    % Draw rectangles on the images indicating the selected region
    if ~isempty(rectangleHandle)
        delete(rectangleHandle);
    end
    if ~isempty(rectangleHandleNoise)
        delete(rectangleHandleNoise);
    end
    if ~isempty(rectangleHandleDenoise)
        delete(rectangleHandleDenoise);
    end
    
    rectangleHandle = rectangle('Parent', originalImageAxes, 'Position', pos, 'EdgeColor', 'r', 'LineWidth', 2);
    rectangleHandleNoise = rectangle('Parent', noisedImageAxes, 'Position', pos, 'EdgeColor', 'r', 'LineWidth', 2);
    rectangleHandleDenoise = rectangle('Parent', denoisedImageAxes, 'Position', pos, 'EdgeColor', 'r', 'LineWidth', 2);

    % Callback function as the slider is moved
    Slider.ValueChangedFcn = @(Slider,event) updateZoom(Slider, croppedImage, croppedNoisedImage, croppedDenoisedImage, croppedImageAxes, croppedNoisedImageAxes, croppedDenoisedImageAxes);
    
    % Short pause for a more responsive interface
    pause(0.5);
    
    % Clear the ROI
    delete(roi);
end

% Callback function to handle zooming
function [] = updateZoom(Slider, croppedImage, croppedNoisedImage, croppedDenoisedImage, croppedImageAxes, croppedNoisedImageAxes, croppedDenoisedImageAxes)
    zoomFactor = Slider.Value;
    
    % Calculate center, width, and height of the ROI for zooming
    centerX = size(croppedImage, 2) / 2;
    centerY = size(croppedImage, 1) / 2;
    width = size(croppedImage, 2) / zoomFactor;
    height = size(croppedImage, 1) / zoomFactor;

    % Set the limits for the axes to achieve the magnification effect
    set(croppedImageAxes, 'XLim', [centerX - width/2, centerX + width/2], 'YLim', [centerY - height/2, centerY + height/2]);
    imshow(croppedImage, 'Parent', croppedImageAxes);
    % title(croppedImageAxes, ['Zoomed Original x', num2str(zoomFactor)]);
    
    set(croppedNoisedImageAxes, 'XLim', [centerX - width/2, centerX + width/2], 'YLim', [centerY - height/2, centerY + height/2]);
    imshow(croppedNoisedImage, 'Parent', croppedNoisedImageAxes);
    % title(croppedNoisedImageAxes, ['Zoomed Noise x', num2str(zoomFactor)]);

    set(croppedDenoisedImageAxes, 'XLim', [centerX - width/2, centerX + width/2], 'YLim', [centerY - height/2, centerY + height/2]);
    imshow(croppedDenoisedImage, 'Parent', croppedDenoisedImageAxes);
    % title(croppedDenoisedImageAxes, ['Zoomed Filtered x', num2str(zoomFactor)]);
end
