%% Setup robot
travelTime = 2; % Defines the travel time
robot = Robot(); % Creates robot object
model = Model();
trajPlanner = Traj_Planner();
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
%% Program 


robot.writeJoints([0, 0, 0 ,0]); % Go to 1st corner

pause(travelTime); % Wait for trajectory completion

fk0 = robot.fk3001([0,0,0,0]);
fk1 = robot.fk3001([0, 0, -90, 0]);

point1 = [fk0(1:3,4)' 0];      %First Corner of Triangle
point2 = [fk1(1:3,4)' 0];  %Second Corner of Triangle

robot.writeJoints(robot.ik3001(point1)); % Go to 1st corner
pause(travelTime / 2);
jointAngle1 = [0,0,0,0];
jointAngle2 = [0,0,-90,0];

t12 = 5;
%P1 to P2
trajPlannerP1toP2Joint1 = trajPlanner.quintic_traj(0, t12, 0, 0, jointAngle1(1,1), jointAngle2(1,1), 0, 0);
trajPlannerP1toP2Joint2 = trajPlanner.quintic_traj(0, t12, 0, 0, jointAngle1(1,2), jointAngle2(1,2), 0, 0);
trajPlannerP1toP2Joint3 = trajPlanner.quintic_traj(0, t12, 0, 0, jointAngle1(1,3), jointAngle2(1,3), 0, 0);
trajPlannerP1toP2Joint4 = trajPlanner.quintic_traj(0, t12, 0, 0, jointAngle1(1,4), jointAngle2(1,4), 0, 0);

P1toP2 = [trajPlannerP1toP2Joint1 trajPlannerP1toP2Joint2 trajPlannerP1toP2Joint3 trajPlannerP1toP2Joint4];
  A = robot.run_trajectory(P1toP2, t12, 0);
   
  Angles = A(1:36, 2:5);
  Time = A(1:36, 1);
  Determ = A(1:36, 6);
  X = [];
  Y = [];
  Z = [];
  for i = 1:36
      FK = robot.fk3001(Angles(i, :));
      X = [X; FK(1,4)];
      Y = [Y; FK(2,4)];
      Z = [Z; FK(3,4)];
  end 

 scatter3(X, Y, Z, '*', 'DisplayName', 'XYZ Plane Movement');
hold on
     title('End effector XYZ values')
     xlabel('X Value (mm)')
     ylabel('Y Value (mm)')
     zlabel('Z Value (mm)')
     %writematrix([Ya4, Ya5, Ya6], 'cubicTask.csv');
     hold off
     legend
     
     figure(3);
     plot(Time, Determ, 'DisplayName', 'Determinant');
     hold on
     title('Determinant Over Time')
     xlabel('Time (seconds)')
     ylabel('Determinant')
     hold off
     legend

  