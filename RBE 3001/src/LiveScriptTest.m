clear 
clc
close all
cam = Camera();
maskPts = [285, 880, 881, 190;
           226, 226, 392, 389];
    try
    
%     pose = cam.getCameraPose();
%     %disp("T_img_check");
%     %disp(cam.cam_pose);
%     try
%                 disp("Put Ball On");
%                 pause;      
%      end
      
     image = imread("ALLCOLORS2.png");
%        image = cam.getImage();
%       image = cam.getImage();

      mask = zeros(size(image, 1), size(image, 2));
      mask = poly2mask(maskPts(1, :), maskPts(2, :), size(image, 1), size(image, 2));
      mask = logical(mask);
      maskedImage = image;
      maskedImage(repmat(~mask, [1, 1, size(image, 3)])) = 0;
      ball = createMask(maskedImage); 
      filledBall = imfill(ball, 'holes');

      ball = logical(ball);
      ballMaskedImage = image;
      
      ballMaskedImage(repmat(~ball, [1, 1, size(image,3)])) = 0;

      L = bwlabel(filledBall);
      stats = regionprops(L, 'Centroid', 'Area');
      validCircles = [];
      for i = 1:length(stats)
        centroid = stats(i).Centroid;
        area = stats(i).Area;
        if area > 1000
            disp(centroid);
            disp(area);
            validCircles = [validCircles; centroid, area];
        end



      end
    colorOfBall = cell(length(validCircles), 1);
    ballColor = cell(length(validCircles), 1);
    for i = 1:length(validCircles)
        hsvImage = rgb2hsv(image);
        [x, y] = meshgrid(1:size(hsvImage, 2), 1:size(hsvImage, 1));
        
        % Extract circle parameters for the current iteration
        centerX = validCircles(i, 1);
        centerY = validCircles(i, 2);
        radius = sqrt(validCircles(i, 3)/pi)/2;
        
        % Create a binary mask for the circle
        maskColor = zeros(size(hsvImage, 1), size(hsvImage, 2));
        maskColor = ((x - centerX).^2 + (y - centerY).^2) <= radius^2;
        maskColor = logical(maskColor);
        % Extract the ROI from the hsvImage
        roiHue = hsvImage(:, :, 1);
        roiSaturation = hsvImage(:, :, 2);
        roiValue = hsvImage(:, :, 3);
        
        roiHue(~maskColor) = 0;
        roiSaturation(~maskColor) = 0;
        roiValue(~maskColor) = 0;
        
        % Calculate the average Hue, Saturation, and Value inside the ROI
        average_hue = mean(roiHue(maskColor));
        average_saturation = mean(roiSaturation(maskColor));
        average_value = mean(roiValue(maskColor));
        
        % Display the average HSV values and debug info
        %fprintf('Circle %d: centerX = %d, centerY = %d, radius = %d\n', i, centerX, centerY, radius);
        %fprintf('Average HSV for Circle %d: Hue = %.2f, Saturation = %.2f, Value = %.2f\n', i, average_hue, average_saturation, average_value);
        
        % You can also convert the average HSV values back to RGB for visualization.
        average_rgb = hsv2rgb([average_hue, average_saturation, average_value]);
        if(average_hue > 0.4)
            ballColorInput = sprintf('Ball %d Color: Green', i);
            ballColor{i} = sprintf('green');
        else
            ballColorInput = sprintf('Ball %d Color: Yellow', i);
            ballColor{i} = sprintf('yellow');
        end
        colorOfBall{i} = ballColorInput;
        
        
        %fprintf('Average RGB for Circle %d: R = %.2f, G = %.2f, B = %.2f\n', i, average_rgb(1), average_rgb(2), average_rgb(3));
    end

     

      BallOverlayed = image;
      for i = 1:length(validCircles)
          BallOverlayed = insertObjectAnnotation(BallOverlayed, "circle", [validCircles(i, 1:2) sqrt(validCircles(i, 3)/pi)], colorOfBall(i),  LineWidth = 2, TextBoxOpacity=0, TextColor = ballColor(i), Color=ballColor(i));
      end
      
      
%       gray_image = im2gray(maskedImage);
%       bw = gray_image < 50;
%       stats = regionprops("table", bw, "Centroid", ...
%           "MajorAxisLength", "MinorAxisLength");

      %imageWithMask = insertObjectMask(image, maskedImage);
      
      figure(1)
      imshow(ballMaskedImage);
%       
%       hold on
%         plot(validCircles(:,1), validCircles(:,2), 'r*')
%       hold off

      catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end