classdef Camera < handle
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties        
        % Properties
        params;     % Camera Parameters
        cam;        % Webcam Object
        cam_pose;   % Camera Pose (transformation matrix)
        cam_IS;     % Camera Intrinsics
        cam_R;      % Camera Rotation Matrix
        cam_T;      % Camera Translation Vector
        cam_TForm   % Camera Rigid 3D TForm
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            % make sure that the webcam can see the whole checkerboard by
            % running webcam(2).preview in the Command Window
            self.cam = webcam(2); % Get camera object
            self.params = self.calibrate(); % Run Calibration Function
            [self.cam_IS, self.cam_pose] = self.calculateCameraPos();
        end

        function tForm = getTForm(self)
            tForm = self.cam_TForm;
        end

        function cam_pose = getCameraPose(self)
            cam_pose = self.cam_pose;
        end

        function cam_IS = getCameraInstrinsics(self)
            cam_IS = self.cam_IS;
        end

        function cam_R = getRotationMatrix(self)
            cam_R = self.cam_R;
        end

        function cam_T = getTranslationVector(self)
            cam_T = self.cam_T;
        end

        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end
      
        function params = calibrate(self)
            % CALIBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camcalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                disp("Calibrating");
                CameraCalibration; % Change this if you are using a different calibration script
                params = cameraParams;
                disp("Camera calibration complete!");
            catch exception
                msg = getReport(exception);
                disp(msg)
                disp("No camera calibration file found. Plese run camera calibration");
            end          
        end
        
        % Returns an undistorted camera image
        function img = getImage(self)
            raw_img =  snapshot(self.cam);
            [img, new_origin] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
        end

        function img = getImageClick(self)
            img = snapshot(self.cam);
        end 

        

        
        function [newIs, pose] = calculateCameraPos(self)  % DO NOT USE
            % calculateCameraPos Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, newIs] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img, 'PartialDetections', false);
            % 4. Compute transformation
            self.params.WorldPoints = self.params.WorldPoints(self.params.WorldPoints(:, 2) <= (boardSize(1)-1)*25, :);
            worldPointSize = size(self.params.WorldPoints);
            imagePointSize = size(imagePoints);
%              fprintf("World Points is %d x %d\n", worldPointSize(1), worldPointSize(2));
%             fprintf("Image Points is %d x %d\n", imagePointSize(1), imagePointSize(2));
%             fprintf("The checkerboard is %d squares long x %d squares wide\n", boardSize(1), boardSize(2));

            % 4. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, newIs);

            self.cam_R = R;
            self.cam_T = t;
            self.cam_TForm = rigid3d([ self.cam_R, zeros(3,1); self.cam_T, 1 ]);
            
            pose = [   R,    t';
                    0, 0, 0, 1];
            
        end


        %%
        %This function maskImageFun takes in an image and a an array. the
        %array a selected amount of points that creates a polygon of the
        %interested part of the field, the image is the current image of
        %the camera.  With this data it first creates the polygon and
        %applies it to the image creating a mask of the area we want.  We
        %then take that area and apply 5 color filters (one for each ball)
        %this returns 5 images that we overlay each other with white spots
        %representing the color balls.  We then take those white spots and
        %do a circle detection on them, giving us the center points as well
        %as the radius.  Using this we can now overlay these circle on our
        %original image.  Once we do that we take the area inside the
        %circle to get our RGB values and then convert the image and get
        %the HSV values.  Then using the HSV values and RGB values we tell
        %what color each ball is. This function returns the image with
        %circles overlayed, the ball coordinates, and the ball colors        
        %%
        function [overlayedImage, coordinates, ballColorToInt] = maskImageFunc(self, image, mask)
               maskPts = mask;
    
               %Polygon Masking Below
              mask = zeros(size(image, 1), size(image, 2));
              mask = poly2mask(maskPts(1, :), maskPts(2, :), size(image, 1), size(image, 2));
              mask = logical(mask);
              maskedImage = image;
              maskedImage(repmat(~mask, [1, 1, size(image, 3)])) = 0;
%               figure(12)
%               imshow(maskedImage);
              %%Color Masking Below
              greenball = GreenBallMask(maskedImage);
              redball = RedBallMask(maskedImage);
              orangeball = OrangeBallMask(maskedImage);
              grayball = GrayBallMask(maskedImage);
              yellowball = YellowBallMask(maskedImage);
              ball = imfuse(yellowball, grayball, 'blend');
              ball = imfuse(ball, orangeball, 'blend');
              ball = imfuse(ball, redball, 'blend');
              ball = imfuse(ball, greenball, 'blend');
%               figure(13)
%               imshow(ball);
              filledBall = imfill(ball, 'holes');
%               figure(14)
%               imshow(filledBall);
        
              ball = logical(ball);
              ballMaskedImage = image;
              
              ballMaskedImage(repmat(~ball, [1, 1, size(image,3)])) = 0;
%               figure(15)
%               imshow(z
%               overlayedImage = ballMaskedImage;
%               coordinates = 1;
%               return;
              %%Finding Circles
              L = bwlabel(filledBall);
              stats = regionprops(L, 'Centroid', 'Area');
              validCircles = [];
              %fprintf('Stats length: %d\n', length(stats))
              k = 1;
              coordinates = [];
              for i = 1:length(stats)
                centroid = stats(i).Centroid;
                area = stats(i).Area;
                %fprintf('Area of thing: %d\n', area)
                %Only take circles we want
                if area > 1400
                    %fprintf("Centroids: %d\n", centroid);
                    %disp(area);
%                     fprintf("Area: %d", area);
                    validCircles(k, :) = [centroid(1) centroid(2) area];
                    coordinates(k, 1) = centroid(1);
                    coordinates(k, 2) = centroid (2);
                    %disp(validCircles(k, :));
                    %fprintf('Valid circles: %d\n', length(validCircles))
                    k = k+1;
                end
              end
              %%BALL Color Below
            %fprintf('Valid circles: %d\n', length(validCircles))
            k = k-1;
            colorOfBall = cell(length(validCircles), 1);
            ballColor = cell(length(validCircles), 1);
            ballColorToInt = zeros(length(validCircles), 1);
            for i = 1:k
                hsvImage = rgb2hsv(image);
%                 figure(20)
%                 imshow(hsvImage);
                [x, y] = meshgrid(1:size(hsvImage, 2), 1:size(hsvImage, 1));
                
                % Extract circle parameters for the current iteration
           
                centerX = validCircles(i, 1);
                centerY = validCircles(i, 2);
                radius = sqrt(validCircles(i, 3)/pi);
                
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

              % fprintf('Average HSV for Circle %d: Hue = %.2f, Saturation = %.2f, Value = %.2f\n', i, average_hue, average_saturation, average_value);
                average_rgb = hsv2rgb([average_hue, average_saturation, average_value]);
                 if(average_hue > 0.41 && average_hue < 0.51 && average_value > 0.38 && average_rgb(2) > 0.35 && average_rgb(1) < 0.20 && average_saturation > 0.7)
                    ballColorInput = sprintf('Ball %d Color: Green', i);
                    ballColor{i} = sprintf('green');
                    %disp("A GREEN BALL")
                    ballColorToInt(i) = 1;
                elseif(average_hue > 0.15 && average_hue < 0.19 && average_value > 0.55 && average_value < 0.85 && average_rgb(3) > 0.25 && average_rgb(3) < 0.4)
                    ballColorInput = sprintf('Ball %d Color: Yellow', i);
                    ballColor{i} = sprintf('yellow'); 
                    %disp("A YELLOW BALL")
                    ballColorToInt(i) = 2;
                elseif(average_hue > 0.05 && average_hue < 0.12 && average_saturation > 0.5 && average_value > 0.4 && average_rgb(3) > 0.2 && average_rgb(3) < 0.3)
                        %Ball is Orange
                        ballColorInput = sprintf('Ball %d Color: Orange', i);
                        ballColor{i} = sprintf('magenta');
                        ballColorToInt(i) = 3;
                elseif(average_hue > 0.45 && average_hue < 0.65 && average_value > 0.40 && average_saturation > 0.2 && average_saturation < 0.6 && average_rgb(2) > 0.25 && average_rgb(2) < 0.65)
                       %Ball is gray
                       ballColorInput = sprintf('Ball %d Color: Gray', i);
                       ballColor{i} = sprintf('cyan');
                       ballColorToInt(i) = 4;
                 elseif(average_saturation  < 0.75 && average_saturation > 0.6 && average_value > 0.45 && average_value < 0.8 && average_rgb(1) > 0.15)
                    %ball is red
                    ballColorInput = sprintf('Ball %d Color: Red', i);
                    ballColor{i} = sprintf('red');
                    ballColorToInt(i) = 5;
                 else
                    ballColorInput = sprintf('Ball %d Color: Unknown', i);
                    ballColor{i} = sprintf('black');
                    ballColorToInt(i) = 6;
                end
                colorOfBall{i} = ballColorInput;
                
                
                %fprintf('Average RGB for Circle %d: R = %.2f, G = %.2f, B = %.2f\n', i, average_rgb(1), average_rgb(2), average_rgb(3));
            end
        
             
              %%Overlay data on original image
              BallOverlayed = image;
              for i = 1:k
                  %  disp(validCircles)
                  BallOverlayed = insertObjectAnnotation(BallOverlayed, "circle", [validCircles(i, 1:2) sqrt(validCircles(i, 3)/pi)], colorOfBall(i),  LineWidth = 2, TextBoxOpacity=0, TextColor = ballColor(i), Color=ballColor(i));
              end
              overlayedImage = BallOverlayed;
%               disp(coordinates);
        end

        %This function takes in the coordinates of the balls and returns
        %the actual coordinates with respect to the robots frame.  We do
        %this with basic geometry using the ball radius and camera height
        %to get the actual x,y coordinates of the center of the ball due to
        %the image being in 2D and us wanting the 3D coordinates
        function ballPts = ballPtsToWorld(self, fakeCoords)
            pose = self.getCameraPose();
            R = pose(1:3, 1:3);
            t = pose(1:3, 4);
            cameraH = 180;
            ballR = 13; %13
            R_0_checker = [ 0  1  0; 1  0  0; 0  0 -1];
            t_0_checker = [113; -92; 0]; %[113; -70; 0]
            T_0_check = [R_0_checker, t_0_checker;zeros(1,3), 1];
            
            amount = size(fakeCoords, 1);
            worldPts = [];
            for i = 1:amount
                worldPts(i, :) = pointsToWorld(self.getCameraInstrinsics, R, t, fakeCoords(i, :));
            end
            
           
            

            ballPts = zeros(amount, 2);
            for i = 1:amount
            r_pos = inv(T_0_check) * [worldPts(i,:)'; 0; 1]; %XY in robot frame
            
            xDiff = ((r_pos(1) * ballR)/cameraH); %Calc X Difference
            robotX = (r_pos(1) + xDiff); %Convert to Robot Frame
            yDiff = r_pos(2)*(robotX - xDiff)/r_pos(1); %Calc Y Difference
            robotY = yDiff;
            ballPts(i, :) = [robotX, robotY];

            end

            %Robot Frame
        end
    end
end