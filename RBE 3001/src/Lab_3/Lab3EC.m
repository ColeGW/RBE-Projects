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

Xc = 250.4;
Yc = 0;
Zc = 224.32;
R = 70;
%Azimuthal angle range:
thetas = linspace(0, 360);

%Polar Angle
phi = 110; 
Xpoints = [];
Ypoints = [];
Zpoints = [];
for theta = thetas
    X = Xc + R * cosd(phi) * cosd(theta);
    Y = Yc + R * sind(phi) * cosd(theta);
    Z = Zc + R*sind(theta);
    Xpoints = [Xpoints, X];
    Ypoints = [Ypoints, Y];
    Zpoints = [Zpoints, Z];
end 
scatter3(Xpoints, Ypoints, Zpoints);
robot.writeJoints(robot.ik3001([Xpoints(1), Ypoints(2), Zpoints(3), 0])); % Go to 1st corner
xlabel("test");
ylabel("test2");
pause(travelTime / 2);
X = [];
Y = [];
Z = [];
time = [];
J1 = [];
J2 = [];
J3 = [];
J4 = [];
tic;
for i = 1:100-1
    
trajPlannerP2toP3X = trajPlanner.quintic_traj(0, 0.05, 0, 0, Xpoints(i), Xpoints(i+1), 0, 0);
trajPlannerP2toP3Y = trajPlanner.quintic_traj(0, 0.05, 0, 0, Ypoints(i), Ypoints(i+1), 0, 0);
trajPlannerP2toP3Z = trajPlanner.quintic_traj(0, 0.05, 0, 0, Zpoints(i), Zpoints(i+1), 0, 0);
trajPlannerP12toP3A = trajPlanner.quintic_traj(0, 0.05, 0, 0, 0, 0, 0, 0);
P1toP2 = [trajPlannerP2toP3X trajPlannerP2toP3Y trajPlannerP2toP3Z trajPlannerP12toP3A];
robot.run_trajectory(P1toP2, 0.05, 1);
angles = robot.setpoint_js;
fk = robot.fk3001(angles);
J1 = [J1; angles(1)];
J2 = [J2; angles(2)];
J3 = [J3; angles(3)];
J4 = [J4; angles(4)];
X =[X; fk(1, 4)];
Y = [Y; fk(2,4)];
Z = [Z; fk(3,4)];
time = [time; toc];
end


% %P1 to P2
% trajPlannerP1toP2X = trajPlanner.cubic_traj(0, 1.8, 0, 0, point1(1,1), point2(1,1));
% trajPlannerP1toP2Y = trajPlanner.cubic_traj(0, 1.8, 0, 0, point1(1,2), point2(1,2));
% trajPlannerP1toP2Z = trajPlanner.cubic_traj(0, 1.8, 0, 0, point1(1,3), point2(1,3));
% 
% P1toP2 = [trajPlannerP1toP2X trajPlannerP1toP2Y trajPlannerP1toP2Z];
% 
% %P2 to P3
% trajPlannerP2toP3X = trajPlanner.cubic_traj(0, 1.7, 0, 0, point2(1,1), point3(1,1));
% trajPlannerP2toP3Y = trajPlanner.cubic_traj(0, 1.7, 0, 0, point2(1,2), point3(1,2));
% trajPlannerP2toP3Z = trajPlanner.cubic_traj(0, 1.7, 0, 0, point2(1,3), point3(1,3));
% 
% P2toP3 = [trajPlannerP2toP3X trajPlannerP2toP3Y trajPlannerP2toP3Z];
% 
% %P3 to P1
% trajPlannerP3toP1X = trajPlanner.cubic_traj(0, 1.7, 0, 0, point3(1,1), point1(1,1));
% trajPlannerP3toP1Y = trajPlanner.cubic_traj(0, 1.7, 0, 0, point3(1,2), point1(1,2));
% trajPlannerP3toP1Z = trajPlanner.cubic_traj(0, 1.7, 0, 0, point3(1,3), point1(1,3));
% 
% P3toP1 = [trajPlannerP3toP1X trajPlannerP3toP1Y trajPlannerP3toP1Z];

%         A = robot.run_trajectory(P1toP2, 1.8);
%         B = robot.run_trajectory(P2toP3, 1.7);
%         C = robot.run_trajectory(P3toP1, 1.7);
%         A = A(1:1650, :);
%         timeOffSetA = A(1650, 1);
%         B = B(1:1600, :);
%         B(:, 1) = B(:, 1) + timeOffSetA; 
%         timeOffSetB = B(1600, 1);
%         C = C(1:1600, :);
%         C(:, 1) = C(:, 1) + timeOffSetB;
%     
%         cutOffMatrix = [A; B; C];
%         cutOffMatrixSize = size(cutOffMatrix);
%         amntRow = cutOffMatrixSize(1);
% 
%       Ya0 = cutOffMatrix(:, 2); %Base joint angle 1
%       Ya1 = cutOffMatrix(:, 3); %Joint angle 1
%       Ya2 = cutOffMatrix(:, 4); %Joint angle 2
%       Ya3 = cutOffMatrix(:, 5); %Joint angle 3
%       Z = zeros(amntRow, 4);
%       xData = [];
%       yData = [];
%       zData = [];
%       for i = 1:amntRow
%         
%         EE = robot.fk3001([Ya0(i), Ya1(i), Ya2(i), Ya3(i)]);  %Create the transformation matrix
%         EEPos = EE(1:3, 4);                    %Trim data for only X, Y, Z locations
%         EEPos = EEPos';                        %Transpose it for easier csv file record
%         xData = [xData; EEPos(1)];
%         yData = [yData; EEPos(2)];
%         zData = [zData; EEPos(3)];
%       end
%        
%      Ya4 = xData; %X Value of End Effector
%      Ya5 = yData; %Y Value of End Effector
%      Ya6 = zData; %Z Value of End Effector
%      X = cutOffMatrix(:, 1); %Time
% 
     % Line plot for XYZ values of end effector
     figure(2);
     subplot(3,1, 1)
     hold on
     plot(time, X, 'LineWidth', 3, 'DisplayName', 'X Value');
     plot(time, Y, 'LineWidth', 3, 'DisplayName', 'Y Value');
     plot(time, Z, 'LineWidth', 3, 'DisplayName', 'Z Value');
     
     grid
     title('End effector XYZ values over time')
     xlabel('Time (seconds)')
     ylabel('Location of end effector (mm)')

     legend
     hold off
     subplot(3,1,2)
        hold on
      plot(time, gradient(X), 'LineWidth', 3, 'DisplayName', 'X Value');
     plot(time, gradient(Y), 'LineWidth', 3, 'DisplayName', 'Y Value');
     plot(time, gradient(Z), 'LineWidth', 3, 'DisplayName', 'Z Value');
     
     
     grid
     title('End effector XYZ Velocities over time')
     xlabel('d/dt (seconds)')
     ylabel('Location of end effector (mm)')

     legend
     hold off
     subplot(3,1,3)
      hold on
      plot(time, gradient(gradient(X)), 'LineWidth', 3, 'DisplayName', 'X Value');
     plot(time, gradient(gradient(Y)), 'LineWidth', 3, 'DisplayName', 'Y Value');
     plot(time, gradient(gradient(Z)), 'LineWidth', 3, 'DisplayName', 'Z Value');
     
  
     grid
     title('End effector XYZ Acceleration over time')
     xlabel('d/dt^2 (seconds)')
     ylabel('Location of end effector (mm)')

     legend
     hold off


     
     % 3D scatter plot for XYZ values of end effector
     figure(3);
     scatter3(X, Y, Z, '*', 'DisplayName', 'XYZ Plane Movement');
     hold on
     scatter3(300, 150, 200, '*', 'DisplayName', 'vertice');
     scatter3(150, 0, 320, '*','DisplayName', 'vertice');
     scatter3(200, 50, 150, '*', 'DisplayName', 'vertice');
     title('End effector XYZ values')
     xlabel('X Value (mm)')
     ylabel('Y Value (mm)')
     zlabel('Z Value (mm)')
     hold off
     legend
     
     % Line plot for joint values
     figure(4);
     plot(time, J1, 'DisplayName', 'Joint 1 Angle');
     hold on
     plot(time, J2, 'DisplayName', 'Joint 2 Angle');
     plot(time, J3, 'DisplayName', 'Joint 3 Angle');
     plot(time, J4, 'DisplayName', 'Joint 4 Angle');
     title('Joint angle values over time')
     xlabel('Time (seconds)')
     ylabel('Joint Angle (Degrees)')
     hold off
     legend

     disp("Done");