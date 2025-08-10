robot = Robot();
model = Model();
travelTime = 1; % Defines the travel time
trajPlanner = Traj_Planner();
robot.writeMotorState(true); % Write position mode

try
    load("camParams.mat");
    disp("Loaded Camera Parameters from camParams.mat");
catch exception
    disp("Could not find camParams.mat, creating new Camera object");
    cam = Camera();
    save("camParams.mat","cam");
    disp("Saved Camera Parameters to camParams.mat");
end

% clickpoint = 1;
% ptInterest = 1;
maskPts = [0, 0, 0, 0;
           0, 0, 0, 0];
robot.writeJoints([-90, 0, 0, 0]);
pause(2);
try
     try
          disp("Put Ball On"); 
          pause;      
     end

image = cam.getImage();


imshow(image);
[x, y] = ginput(4);
maskPts = [x'; y'];

image = cam.getImage();
[overlayImage, circleData, ballColorToInt] = cam.maskImageFunc(image, maskPts);
figure(2);
imshow(overlayImage) 
 ballPts = cam.ballPtsToWorld(circleData(:, 1:2));
 amountBalls = size(ballPts);
 amountBalls = amountBalls(1);
    ikHome = [-90, 0, 0, 0];

    ik = robot.ik3001([ballPts(1, :), 100, 90]);
     robot.writeGripper(1);
     trajPlannerQ1 = trajPlanner.cubic_traj(0, 1, 0, 0, ikHome(1), ik(1));
     trajPlannerQ2 = trajPlanner.cubic_traj(0, 1, 0, 0, ikHome(2), ik(2));
     trajPlannerQ3 = trajPlanner.cubic_traj(0, 1, 0, 0, ikHome(3), ik(3));
     trajPlannerQ4 = trajPlanner.cubic_traj(0, 1, 0, 0, ikHome(4), ik(4));
     PHtoPAbove = [trajPlannerQ1 trajPlannerQ2 trajPlannerQ3 trajPlannerQ4];
     robot.run_trajectory(PHtoPAbove, 1, 0);
     ikOld = ik;
     pause(2);
    while(1)
     image = cam.getImage();
     [overlayImage, circleData, ballColorToInt] = cam.maskImageFunc(image, maskPts);
     if(size(circleData) > 0)
        ballPts = cam.ballPtsToWorld(circleData(1:2));
        ikNew = robot.ik3001([ballPts(1, 1), ballPts(1, 2), 100, 90]);
        robot.interpolate_jp(ikNew, 1);
     end
%         test = size(circleData);
%     if(test > 0)
%         figure(2)
% %         imshow(overlayImage) 
%         
%         amountBalls = size(ballPts);
%         amountBalls = amountBalls(1);
%     end
    end
    


catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end