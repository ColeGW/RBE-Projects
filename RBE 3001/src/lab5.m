clc;
clear;
clc;
close all
robot = Robot();
model = Model();
cam = Camera();
clickpoint = 0;
ptInterest = 1;
maskPts = [289, 759, 881, 180;
           206, 206, 392, 389];
try
    
    pose = cam.getCameraPose();
    %disp("T_img_check");
    %disp(cam.cam_pose);
    
    
    R = pose(1:3, 1:3);
    t = pose(1:3, 4);
    if(clickpoint == 1)
        imshow(cam.getImage());
        [x, y] = ginput(4);
        xy42 = [x, y];
        worldPt = pointsToWorld(cam.getCameraInstrinsics(), R, t, xy42);
    else
        image = cam.getImage();
        [imagePoints, boardSize] =  detectCheckerboardPoints(image);
        J = insertText(image, imagePoints, 1:size(imagePoints,1));
        J = insertMarker(J, imagePoints, 'o','Color', 'red', 'Size', 5);
        figure(4);
        imshow(J);
        title(sprintf('Detected a %d x %d Checkerboard', boardSize));
        worldPt = pointsToWorld(cam.getCameraInstrinsics(), R, t, imagePoints);
        figure(6);
        scatter(worldPt(:,1), worldPt(:, 2));
        title("Image Points");
    end

    

    %disp(worldPt);
    R_0_checker = [ 0  1  0; 1  0  0; 0  0 -1];
    t_0_checker = [113; -92; 0]; %[113; -70; 0]
    T_0_check = [R_0_checker, t_0_checker;zeros(1,3), 1];
    
    if(clickpoint == 1)
    r1_pos = inv(T_0_check) * [worldPt(1,:)'; 0; 1];
    r2_pos = inv(T_0_check) * [worldPt(2,:)'; 0; 1];
    r3_pos = inv(T_0_check) * [worldPt(3,:)'; 0; 1];
    r4_pos = inv(T_0_check) * [worldPt(4,:)'; 0; 1];
    else 
        inv_T_0_check = inv(T_0_check);
        r_pos = inv(T_0_check) * [worldPt(ptInterest, :)'; 0; 1];
        r_pos(4) = 90;
        angles = robot.ik3001(r_pos);
       figure(5);
        model.plot_arm(angles);
    end

     try
                disp("Put Ball On");
                pause;      
     end
  
while(1)
      image = cam.getImage();
      [overlayImage, circleData] = cam.maskImageFunc(image);
      mask = zeros(size(image, 1), size(image, 2));
      mask = poly2mask(maskPts(1, :), maskPts(2, :), size(image, 1), size(image, 2));
      mask = logical(mask);
      maskedImage = image;
      maskedImage(repmat(~mask, [1, 1, size(image, 3)])) = 0;
      ball = createMask(maskedImage); 
      ball = imfill(ball, 'holes');
      L = bwlabel(ball);
      stats = regionprops(L, 'Centroid', 'Area');
      disp("DEEZ")  
      for i = 1:length(stats)
        centroid = stats(i).Centroid;
        area = stats(i).Area;
        if area > 3000
            disp(centroid);
        end
      end
      
      gray_image = im2gray(maskedImage);
      bw = gray_image < 50;
      stats = regionprops("table", bw, "Centroid", ...
          "MajorAxisLength", "MinorAxisLength");

      imageWithMask = insertObjectMask(image, maskedImage);
      figure(10);
      imshow(overlayImage);
%       hold on
%       
%       hold off
%     r_pos = centers_to_positions(r_pos(1:2)');
      
end
 
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
