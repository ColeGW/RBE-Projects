%% Setup robot
close all
travelTime = 6; % Defines the travel time
robot = Robot(); % Creates robot object
trajPlanner = Traj_Planner();
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
%% Program 
robot.writeJoints(0);
point1 = [300, 150, 200, 0];      %First Corner of Triangle
point2 = [150, 0, 320, 0];  %Second Corner of Triangle
point3 = [200, 50, 150, 0];    %Third Corner of Triange

robot.writeJoints(robot.ik3001(point1)); % Go to 1st corner
pause(travelTime / 2);
jointAngle1 = point1;
jointAngle2 = point2;
jointAngle3 = point3;

%P1 to P2
trajPlannerP1toP2X = trajPlanner.cubic_traj(0, 1.8, 0, 0, point1(1,1), point2(1,1));
trajPlannerP1toP2Y = trajPlanner.cubic_traj(0, 1.8, 0, 0, point1(1,2), point2(1,2));
trajPlannerP1toP2Z = trajPlanner.cubic_traj(0, 1.8, 0, 0, point1(1,3), point2(1,3));
trajPlannerP1toP2A = trajPlanner.cubic_traj(0, 1.8, 0, 0, point1(1,4), point2(1,4));
P1toP2 = [trajPlannerP1toP2X trajPlannerP1toP2Y trajPlannerP1toP2Z trajPlannerP1toP2A];

%P2 to P3
trajPlannerP2toP3X = trajPlanner.cubic_traj(0, 1.7, 0, 0, point2(1,1), point3(1,1));
trajPlannerP2toP3Y = trajPlanner.cubic_traj(0, 1.7, 0, 0, point2(1,2), point3(1,2));
trajPlannerP2toP3Z = trajPlanner.cubic_traj(0, 1.7, 0, 0, point2(1,3), point3(1,3));
trajPlannerP12toP3A = trajPlanner.cubic_traj(0, 1.7, 0, 0, point2(1,4), point3(1,4));
P2toP3 = [trajPlannerP2toP3X trajPlannerP2toP3Y trajPlannerP2toP3Z trajPlannerP12toP3A];

%P3 to P1
trajPlannerP3toP1X = trajPlanner.cubic_traj(0, 1.7, 0, 0, point3(1,1), point1(1,1));
trajPlannerP3toP1Y = trajPlanner.cubic_traj(0, 1.7, 0, 0, point3(1,2), point1(1,2));
trajPlannerP3toP1Z = trajPlanner.cubic_traj(0, 1.7, 0, 0, point3(1,3), point1(1,3));
trajPlannerP13toP1A = trajPlanner.cubic_traj(0, 1.7, 0, 0, point3(1,4), point1(1,4));
P3toP1 = [trajPlannerP3toP1X trajPlannerP3toP1Y trajPlannerP3toP1Z trajPlannerP13toP1A];
        task = 1;
        A = robot.run_trajectory(P1toP2, 1.8, task);
        B = robot.run_trajectory(P2toP3, 1.7, task);
        C = robot.run_trajectory(P3toP1, 1.7, task);
        A = A(1:1650, :);
        timeOffSetA = A(1650, 1);
        B = B(1:1600, :);
        B(:, 1) = B(:, 1) + timeOffSetA; 
        timeOffSetB = B(1600, 1);
        C = C(1:1600, :);
        C(:, 1) = C(:, 1) + timeOffSetB;
    
        cutOffMatrix = [A; B; C];
        cutOffMatrixSize = size(cutOffMatrix);
        amntRow = cutOffMatrixSize(1);

      Ya0 = cutOffMatrix(:, 2); %Base joint angle 1
      Ya1 = cutOffMatrix(:, 3); %Joint angle 1
      Ya2 = cutOffMatrix(:, 4); %Joint angle 2
      Ya3 = cutOffMatrix(:, 5); %Joint angle 3
      Z = zeros(amntRow, 4);
      xData = [];
      yData = [];
      zData = [];
      for i = 1:amntRow
        
        EE = robot.fk3001([Ya0(i), Ya1(i), Ya2(i), Ya3(i)]);  %Create the transformation matrix
        EEPos = EE(1:3, 4);                    %Trim data for only X, Y, Z locations
        EEPos = EEPos';                        %Transpose it for easier csv file record
        xData = [xData; EEPos(1)];
        yData = [yData; EEPos(2)];
        zData = [zData; EEPos(3)];
      end
       
     Ya4 = xData; %X Value of End Effector
     Ya5 = yData; %Y Value of End Effector
     Ya6 = zData; %Z Value of End Effector
     X = cutOffMatrix(:, 1); %Time

     % Line plot for XYZ values of end effector
     figure(2);
     subplot(3,1, 1)
     hold on
     plot(X, Ya4, 'LineWidth', 3, 'DisplayName', 'X Value');
     plot(X, Ya5, 'LineWidth', 3, 'DisplayName', 'Y Value');
     plot(X, Ya6, 'LineWidth', 3, 'DisplayName', 'Z Value');
     
     grid
     title('End effector XYZ values over time')
     xlabel('Time (seconds)')
     ylabel('Location of end effector (mm)')

     legend
     hold off
     subplot(3,1,2)
        hold on
      plot(X, gradient(Ya4), 'LineWidth', 3, 'DisplayName', 'X Value');
     plot(X, gradient(Ya5), 'LineWidth', 3, 'DisplayName', 'Y Value');
     plot(X, gradient(Ya6), 'LineWidth', 3, 'DisplayName', 'Z Value');
     
     
     grid
     title('End effector XYZ Velocities over time')
     xlabel('d/dt (seconds)')
     ylabel('Location of end effector (mm)')

     legend
     hold off
     subplot(3,1,3)
      hold on
      plot(X, gradient(gradient(Ya4)), 'LineWidth', 3, 'DisplayName', 'X Value');
     plot(X, gradient(gradient(Ya5)), 'LineWidth', 3, 'DisplayName', 'Y Value');
     plot(X, gradient(gradient(Ya6)), 'LineWidth', 3, 'DisplayName', 'Z Value');
     
  
     grid
     title('End effector XYZ Acceleration over time')
     xlabel('d/dt^2 (seconds)')
     ylabel('Location of end effector (mm)')

     legend
     hold off


     
     % 3D scatter plot for XYZ values of end effector
     figure(3);
     scatter3(Ya4, Ya5, Ya6, '*', 'DisplayName', 'XYZ Plane Movement');
     hold on
     scatter3(300, 150, 200, '*', 'DisplayName', 'vertice');
     scatter3(150, 0, 320, '*','DisplayName', 'vertice');
     scatter3(200, 50, 150, '*', 'DisplayName', 'vertice');
     title('End effector XYZ values')
     xlabel('X Value (mm)')
     ylabel('Y Value (mm)')
     zlabel('Z Value (mm)')
     %writematrix([Ya4, Ya5, Ya6], 'cubicTask.csv');
     hold off
     legend
     
     % Line plot for joint values
     figure(4);
     plot(X, Ya0, 'DisplayName', 'Joint 1 Angle');
     hold on
     plot(X, Ya1, 'DisplayName', 'Joint 2 Angle');
     plot(X, Ya2, 'DisplayName', 'Joint 3 Angle');
     plot(X, Ya3, 'DisplayName', 'Joint 4 Angle');
     title('Joint angle values over time')
     xlabel('Time (seconds)')
     ylabel('Joint Angle (Degrees)')
     hold off
     legend

     disp("Done");