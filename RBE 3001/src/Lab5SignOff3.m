robot = Robot();
model = Model();

try
    load("camParams.mat");
    disp("Loaded Camera Parameters from camParams.mat");
catch exception
    disp("Could not find camParams.mat, creating new Camera object");
    cam = Camera();
    save("camParams.mat","cam");
    disp("Saved Camera Parameters to camParams.mat");
end

clickpoint = 1;
ptInterest = 1;
maskPts = [0, 0, 0, 0;
           0, 0, 0, 0];
try
    %%Height of Camera = 175mm
  image = cam.getImage();
% image = imread("ALLCOLORS2.png");

imshow(image);
[x, y] = ginput(4);
maskPts = [x'; y'];
 try
          disp("Put Ball On"); 
          pause;      
 end
while(1)
     image = cam.getImage();
    [overlayImage, circleData] = cam.maskImageFunc(image, maskPts);
    figure(2);
    imshow(overlayImage) 
    ballPts = cam.ballPtsToWorld(maskPts(:, 1:2));
end

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end