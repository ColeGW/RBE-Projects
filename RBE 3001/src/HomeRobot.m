%% Setup robot
travelTime = 5; % Defines the travel time
robot = Robot(); % Creates robot object
%robot.writeTime(travelTime); % Write travel time
%robot.writeMotorState(true); % Write position mode
robot.writeGripper(1);
% robot.writeJoints([0, 0, 0, 0]);
robot.writeJoints([-90, -86, 90, 33]);
