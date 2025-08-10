%% Setup robot
robot = Robot(); % Creates robot object
angles0 = [ 0; 0; 0; 0];
angles10 = [10; 10; 10; 10];
angles510 = [-5, -10, -5, -10];
robot.servo_jp(angles10);

travelTime = 1; % Sets travel time
robot.writeTime(travelTime); % Write travel time
bufferTime = 0; % Sets buffertime or something man idk
aCounter = 5; % Runs loop 5 times

tic;

robot.servo_jp(angles0); % Sends to home point to begin

while (toc - bufferTime < travelTime) % Waits for arm to move
end


for i = 1:aCounter
    bufferTime = toc; % Records time to buffer to allow arm movement

    robot.servo_jp(angles10); % Sends to arbitrary point
    
    while (toc - bufferTime) < travelTime % Waits for arm to move
    end
    
    bufferTime = toc; % Records time to buffer to allow arm movement
    
    robot.servo_jp(angles0); % Return to home point
    
    while (toc - bufferTime) < travelTime % Waits for arm to move
    end
    
    Tactual = robot.fk3001(angles0);    
    if i == 1
        T1 = robot.measured_cp(); % Records measured tip position
    elseif i == 2
        T2 = robot.measured_cp(); % Records measured tip position  
    elseif i == 3
        T3 = robot.measured_cp(); % Records measured tip position
    elseif i == 4
        T4 = robot.measured_cp(); % Records measured tip position
    else
        T5 = robot.measured_cp(); % Records measured tip position
    end

end
% Sends robot Home
robot.writeJoints([-90, -86, 90, 33]);

% Plots the robots actual points from our FK caculation and T1-T5, this
% indicates the 5 movemments from [0, 0, 0, 0] - [10, 10, 10, 10]
robot.plott(T1,T2,T3,T4,T5,Tactual)

% Caculates the RMSE from the actual and T1-T5
x = [T1(1,4); T2(1,4); T3(1,4); T4(1,4); T5(1,4)];
y = [T1(2,4); T2(2,4); T3(2,4); T4(2,4); T5(2,4)];
z = [T1(3,4); T2(3,4); T3(3,4); T4(3,4); T5(3,4)];
xactual = zeros(5, 1);
yactual = zeros(5, 1);
zactual = zeros(5, 1);
xactual(1:5, 1) = Tactual(1,4);
yactual(1:5, 1) = Tactual(2,4);
zactual(1:5, 1) = Tactual(3,4);

% Displays the RMSE to verify
%disp(robot.rmse(x, xactual))
%disp(robot.rmse(y, yactual))
%disp(robot.rmse(z, zactual))

% Creates a 4x1 matrix called RMSE and writes it to a csv file
RMSE = [robot.rmse(x, xactual); robot.rmse(y, yactual); robot.rmse(z, zactual)];

writematrix(RMSE, 'RMSE.csv');



