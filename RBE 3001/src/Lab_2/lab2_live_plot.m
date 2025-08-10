%% Setup robot
travelTime = 10; % Defines the travel time
robot = Robot(); % Creates robot object
model = Model(); % Creates model object
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
%% Program 
robot.writeJoints(0); % Write joints to zero position
pause(travelTime/2); % Wait for trajectory completion


    %Tell the robot a random point to go to
    robot.interpolate_jp([45, -40, 20, 10], travelTime*1000); % Write joint values to robot to start movement

    tic; % Start timer
    while toc < travelTime
        model.plot_arm(robot.setpoint_js()); % Read joint values into model plot so that it live plots
        
    end
%{

%% CODE FOR EXTRA CREDIT

Limits are z > L0 which is 0s
xBound = zeros(100000, 1);
yBound = zeros(100000, 1);
zBound = zeros(100000, 1);
boundsCounter = 1;
for i = -90:10:90
    for j = -180:10:180
        for k = -180:10:180
            for l = -180:10:180
                T = fk3001([i, j, k, l]);
                Trimmed = T(1:3, 4);
                if(Trimmed(3,1) > 0)
                    xBound(boundsCounter) = Trimmed(1,1);
                    yBound(boundsCounter) = Trimmed(2,1);
                    zBound(boundsCounter) = Trimmed(3,1);
                    boundsCounter = boundsCounter + 1;
                end 
            end
        end
    end
end

k = boundary(xBound, yBound, zBound);
grid on
plot3( xBound(k), yBound(k),zBound(k));
grid on
xlabel("X-Axis(mm)");
ylabel("Y-Axis(mm)");
zlabel("Z-Axis(mm)");
title("Robots Workspace");

%}