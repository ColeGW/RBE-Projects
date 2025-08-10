robot = Robot();
model = Model();
trajPlanner = Traj_Planner();
robot.writeMotorState(true); % Write position mode

%%Create camParams.mat if not found otherwise camera already calibrated
try
    load("camParams.mat");
    disp("Loaded Camera Parameters from camParams.mat");
catch exception
    disp("Could not find camParams.mat, creating new Camera object");
    cam = Camera();
    save("camParams.mat","cam");
    disp("Saved Camera Parameters to camParams.mat");
    
end

%%Set up polygon of interest
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

%%Click points for polygon
imshow(image);
[x, y] = ginput(5);
maskPts = [x'; y'];

image = cam.getImage();
[overlayImage, circleData, ballColorToInt] = cam.maskImageFunc(image, maskPts);
figure(2);
imshow(overlayImage) 

figure(3)
imshow(overlayImage)
hold on
plot(circleData(:, 1), circleData(:,2), 'r+');
hold off
 ballPts = cam.ballPtsToWorld(circleData(:, 1:2));
 amountBalls = size(ballPts);
 amountBalls = amountBalls(1);

while(amountBalls > 0)
    ballR = 13;
    ik = robot.ik3001([ballPts(amountBalls, :), 224, 0]);
    
    %%Go above Ball
     ikHome = [-90, 0, 0, 0];
     robot.writeGripper(1);
     pause(2);
     trajPlannerQ1 = trajPlanner.cubic_traj(0, 2, 0, 0, ikHome(1), ik(1));
     trajPlannerQ2 = trajPlanner.cubic_traj(0, 2, 0, 0, ikHome(2), ik(2));
     trajPlannerQ3 = trajPlanner.cubic_traj(0, 2, 0, 0, ikHome(3), ik(3));
     trajPlannerQ4 = trajPlanner.cubic_traj(0, 2, 0, 0, ikHome(4), ik(4));
     PHtoPAbove = [trajPlannerQ1 trajPlannerQ2 trajPlannerQ3 trajPlannerQ4];
     robot.run_trajectory(PHtoPAbove, 2, 0);
     pause(2);
    
     %%Go to Ball
     ikDown = robot.ik3001([ballPts(amountBalls, :), 15, 90]);
     trajPlannerQ1 = trajPlanner.cubic_traj(0, 1, 0, 0, ik(1), ikDown(1));
     trajPlannerQ2 = trajPlanner.cubic_traj(0, 1, 0, 0, ik(2), ikDown(2));
     trajPlannerQ3 = trajPlanner.cubic_traj(0, 1, 0, 0, ik(3), ikDown(3));
     trajPlannerQ4 = trajPlanner.cubic_traj(0, 1, 0, 0, ik(4), ikDown(4));
     PHtoPDown = [trajPlannerQ1 trajPlannerQ2 trajPlannerQ3 trajPlannerQ4];
     robot.run_trajectory(PHtoPDown, 1, 0);
     pause(2);

     %%Go above Ball after picking up
     robot.writeGripper(0);
     pause(2);
     trajPlannerQ1 = trajPlanner.cubic_traj(0, 1, 0, 0, ikDown(1), ik(1));
     trajPlannerQ2 = trajPlanner.cubic_traj(0, 1, 0, 0, ikDown(2), ik(2));
     trajPlannerQ3 = trajPlanner.cubic_traj(0, 1, 0, 0, ikDown(3), ik(3));
     trajPlannerQ4 = trajPlanner.cubic_traj(0, 1, 0, 0, ikDown(4), ik(4));
     PHtoPUp = [trajPlannerQ1 trajPlannerQ2 trajPlannerQ3 trajPlannerQ4];
     robot.run_trajectory(PHtoPUp, 1, 0);

     if(ballColorToInt(amountBalls) == 1 ) %Green
         ikEnd = [-90, 0, 0, 90];
     elseif(ballColorToInt(amountBalls) == 2) %Yellow
         ikEnd = [-90, 0, 0, -45];
     elseif(ballColorToInt(amountBalls) == 3) %Orange
         ikEnd = [90, 0, 0, 90];
     elseif(ballColorToInt(amountBalls) == 4) %Gray
         ikEnd = [90, 0, 0, -45];
     elseif(ballColorToInt(amountBalls) == 5) %Red
         ikEnd = [90, 0, 0, 0];
     else %Unknown
         ikEnd = ikHome;
     end
     
     %%Go to drop off point
     trajPlannerQ1 = trajPlanner.cubic_traj(0, 2, 0, 0, ik(1), ikEnd(1));
     trajPlannerQ2 = trajPlanner.cubic_traj(0, 2, 0, 0, ik(2), ikEnd(2));
     trajPlannerQ3 = trajPlanner.cubic_traj(0, 2, 0, 0, ik(3), ikEnd(3));
     trajPlannerQ4 = trajPlanner.cubic_traj(0, 2, 0, 0, ik(4), ikEnd(4));
     PHtoPEnd = [trajPlannerQ1 trajPlannerQ2 trajPlannerQ3 trajPlannerQ4];
     robot.run_trajectory(PHtoPEnd, 2, 0);
     pause(2);
     robot.writeGripper(1);

     %%Return Home
     trajPlannerQ1 = trajPlanner.cubic_traj(0, 2, 0, 0, ikEnd(1), ikHome(1));
     trajPlannerQ2 = trajPlanner.cubic_traj(0, 2, 0, 0, ikEnd(2), ikHome(2));
     trajPlannerQ3 = trajPlanner.cubic_traj(0, 2, 0, 0, ikEnd(3), ikHome(3));
     trajPlannerQ4 = trajPlanner.cubic_traj(0, 2, 0, 0, ikEnd(4), ikHome(4));
     PHtoPHome = [trajPlannerQ1 trajPlannerQ2 trajPlannerQ3 trajPlannerQ4];
     robot.run_trajectory(PHtoPHome, 2, 0);
     pause(2);


     amountBalls = amountBalls - 1;
    %%Redo image to see if ball has been replaced
    image = cam.getImage();
    [overlayImage, circleData, ballColorToInt] = cam.maskImageFunc(image, maskPts);
    test = size(circleData);
    if(test > 0)
        figure(2)
        imshow(overlayImage) 
        ballPts = cam.ballPtsToWorld(circleData(:, 1:2));
        amountBalls = size(ballPts);
        amountBalls = amountBalls(1);
    end
end 

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end