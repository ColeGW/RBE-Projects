%% Setup robot
travelTime = 5; % Defines the travel time
robot = Robot(); % Creates robot object
model = Model();
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
%% Program 

robot.writeJoints(0); % Write joints to zero position
pause(travelTime); % Wait for trajectory completion
point1 = [300, 150, 200, 0];      %First Corner of Triangle
point2 = [150, 0, 320, 0];  %Second Corner of Triangle
point3 = [200, 50, 150, 0];    %Third Corner of Triange


    

Z = robot.jacob3001([35, -170, 90, 0])
XYZ = Z(1:3, 1:3);
disp(XYZ);
disp(det(XYZ));