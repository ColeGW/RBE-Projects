%% Setup robot
travelTime = 1; % Defines the travel time
robot = Robot(); % Creates robot object
model = Model();
trajPlanner = Traj_Planner();
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
%% Program 

%Below if SignOff 2 which is commented out so we dont have to keep running
%to make the plots perfect merely uncomment:
%{
robot.writeJoints([0, 0, 0, 0]); % Write joints to zero position
pause(travelTime); % Wait for trajectory completion

point1 = [300, 150, 200, 0];      %First Corner of Triangle
point2 = [150, 0, 320, 0];  %Second Corner of Triangle
point3 = [200, 50, 150, 0];    %Third Corner of Triange

robot.writeJoints(robot.ik3001(point1)); % Go to 1st corner
pause(travelTime / 2);
jointAngle1 = robot.ik3001(point1);
jointAngle2 = robot.ik3001(point2);
jointAngle3 = robot.ik3001(point3);
t12 = 5;
t23 = 5;
t31 = 5;
%P1 to P2
trajPlannerP1toP2Joint1 = trajPlanner.quintic_traj(0, t12, 0, 0, jointAngle1(1,1), jointAngle2(1,1), 0, 0);
trajPlannerP1toP2Joint2 = trajPlanner.quintic_traj(0, t12, 0, 0, jointAngle1(1,2), jointAngle2(1,2), 0, 0);
trajPlannerP1toP2Joint3 = trajPlanner.quintic_traj(0, t12, 0, 0, jointAngle1(1,3), jointAngle2(1,3), 0, 0);
trajPlannerP1toP2Joint4 = trajPlanner.quintic_traj(0, t12, 0, 0, jointAngle1(1,4), jointAngle2(1,4), 0, 0);

P1toP2 = [trajPlannerP1toP2Joint1 trajPlannerP1toP2Joint2 trajPlannerP1toP2Joint3 trajPlannerP1toP2Joint4];

%P2 to P3
trajPlannerP2toP3Joint1 = trajPlanner.quintic_traj(0, t23, 0, 0, jointAngle2(1,1), jointAngle3(1,1), 0, 0);
trajPlannerP2toP3Joint2 = trajPlanner.quintic_traj(0, t23, 0, 0, jointAngle2(1,2), jointAngle3(1,2), 0, 0);
trajPlannerP2toP3Joint3 = trajPlanner.quintic_traj(0, t23, 0, 0, jointAngle2(1,3), jointAngle3(1,3), 0, 0);
trajPlannerP2toP3Joint4 = trajPlanner.quintic_traj(0, t23, 0, 0, jointAngle2(1,4), jointAngle3(1,4), 0, 0);
P2toP3 = [trajPlannerP2toP3Joint1 trajPlannerP2toP3Joint2 trajPlannerP2toP3Joint3 trajPlannerP2toP3Joint4];

%P3 to P2
trajPlannerP3toP1Joint1 = trajPlanner.quintic_traj(0, t31, 0, 0, jointAngle3(1,1), jointAngle1(1,1), 0, 0);
trajPlannerP3toP1Joint2 = trajPlanner.quintic_traj(0, t31, 0, 0, jointAngle3(1,2), jointAngle1(1,2), 0, 0);
trajPlannerP3toP1Joint3 = trajPlanner.quintic_traj(0, t31, 0, 0, jointAngle3(1,3), jointAngle1(1,3), 0, 0);
trajPlannerP3toP1Joint4 = trajPlanner.quintic_traj(0, t31, 0, 0, jointAngle3(1,4), jointAngle1(1,4), 0, 0);
P3toP1 = [trajPlannerP3toP1Joint1 trajPlannerP3toP1Joint2 trajPlannerP3toP1Joint3 trajPlannerP3toP1Joint4];
        


        

        A = robot.run_trajectoryPLOT(P1toP2, t12, 0, model, 1);
        L = readmatrix("Jacob.csv");
        B = robot.run_trajectoryPLOT(P2toP3, t23, 0, model, 2);
        L = readmatrix("Jacob.csv");
        C = robot.run_trajectoryPLOT(P3toP1, t31, 0, model, 3);
        A = A(1:50, :);
        timeOffSetA = A(50, 1);
        B = B(1:45, :);
        B(:, 1) = B(:, 1) + timeOffSetA; 
        timeOffSetB = B(45, 1);
        C = C(1:45, :);
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
        
%}
        cutoff = 105;
        J1 = readmatrix('Jacob1.csv');
        J2 = readmatrix('Jacob2.csv');
        J3 = readmatrix('Jacob3.csv');
        
        J1 = [J1(:, 1:6) J1(:, 7) - J1(1,7)];

        J2 = [J2(:, 1:6) (J2(:, 7) - J2(1,7) + J1(cutoff, 7))];
    
        J3 = [J3(:, 1:6) (J3(:, 7) - J3(1,7) + J2(cutoff, 7))];
        

        J = [J1; J2; J3];
        
        XVel1 = J(1:cutoff, 1);
        YVel1 = J(1:cutoff, 2);
        ZVel1 = J(1:cutoff, 3);
        XAngVel1 = J(1:cutoff, 4);
        YAngVel1 = J(1:cutoff, 5);
        ZAngVel1 = J(1:cutoff, 6);
        T1 = J(1:cutoff, 7);

        figure(2);
        plot(T1, XVel1, 'LineWidth', 3, 'DisplayName', 'X Vel', 'Color', 'r');
        hold on 
        plot(T1, YVel1, 'LineWidth', 3, 'DisplayName', 'Y Vel', 'Color', 'b');
        plot(T1, ZVel1, 'LineWidth', 3, 'DisplayName', 'Z Vel', 'Color', 'g');
        hold off
        grid
        title('End effector XYZ linear velocities over time')
        xlabel('Time (seconds)')
        ylabel('Linear Velocity of End Effector (mm/s)')
        legend

        figure(3);
        plot(T1, XAngVel1, 'LineWidth', 3, 'DisplayName', 'X Vel', 'Color', 'r');
        hold on 
        plot(T1, YAngVel1, 'LineWidth', 3, 'DisplayName', 'Y Vel', 'Color', 'b');
        plot(T1, ZAngVel1, 'LineWidth', 3, 'DisplayName', 'Z Vel', 'Color', 'g');
        hold off
        grid
        title('End effector XYZ angular velocities over time')
        xlabel('Time (seconds)')
        ylabel('Angular Velocity of End Effector (degrees/s)')
        legend
                
        figure(4);
        plot(T1, abs(XVel1), 'LineWidth', 3, 'DisplayName', 'X Vel', 'Color', 'r');
        hold on 
        plot(T1, abs(YVel1), 'LineWidth', 3, 'DisplayName', 'Y Vel', 'Color', 'b');
        plot(T1, abs(ZVel1), 'LineWidth', 3, 'DisplayName', 'Z Vel', 'Color', 'g');
        hold off
        grid
        title('End effector XYZ magnitude of velocities over time')
        xlabel('Time (seconds)')
        ylabel('Magnitude of Velocity of End Effector (mm/s)')
        legend

%{       
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
%}
     disp("Done");

        