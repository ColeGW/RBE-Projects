%% Setup robot
travelTime = 5; % Defines the travel time
robot = Robot(); % Creates robot object
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(0); % Write position mode
%% Program 

while(1)
joints = robot.setpoint_js();
fk = robot.fk3001(joints);
fk = fk(1:3, 4);
disp(fk')
end