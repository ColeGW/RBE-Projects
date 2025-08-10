%% Setup robot
close all
travelTime = 10; % Defines the travel time
robot = Robot(); % Creates robot object
traj = Traj_Planner(); % Creates trajectory planner object
robot.writeMotorState(true); % Write position mode

%% Program 
robot.writeJoints(0);
point1 = [300, 150, 200, 0];      %First Corner of Triangle
point2 = [150, 0, 320, 0];  %Second Corner of Triangle
point3 = [200, 50, 150, 0];    %Third Corner of Triange


one_two = traj.cubic_traj(0, 1.8, 0, 0, point1, point2);
two_three = traj.cubic_traj(1.8, 3.5, 0, 0, point2, point3);
three_one = traj.cubic_traj(3.5, 5.2, 0, 0, point3, point1);

robot.run_trajectory(one_two, 1.8);
robot.run_trajectory(two_three, 1.7);
robot.run_trajectory(three_one, 1.7);

%{
%POINT 1 TO POINT 2
%1.768 seconds

%POINT 2 TO POINT 3
%1.6270 seconds

%POINT 3 TO POINT 1
%1.675 seconds

T = robot.ik3001(point1);
run = 1;

robot.servo_jp(robot.ik3001(point3));
pause(4);
disp("Start");
tic;
robot.servo_jp(robot.ik3001(point1));
while (run == 1)
    r = robot.getJointsReadings();
    if r(1, 1) > T(1, 1) - 2 & r(1, 1) < T(1, 1) + 2
        disp("End");
        disp(toc);
        run = 0;
    end
end

%}