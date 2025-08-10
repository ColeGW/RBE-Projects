%% Setup robot
travelTime = 10; % Defines the travel time
robot = Robot(); % Creates robot object
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
%% Program 

robot.writeJoints(0); % Write joints to zero position
pause(travelTime/2); % Wait for trajectory completion


%zx
point1 = [0, 0, 0, 0];      %First Corner of Triangle
point2 = [0, -30, -30, 0];  %Second Corner of Triangle
point3 = [0, -50, 0, 0];    %Third Corner of Triange
jointPosition = zeros(10000, 8);
    counter = 1;   %counter is to iterate for the CSV table to put data into a matrix
    counter2 = 1;  %counter2 is to iterate through robot movements so that the function is only ever called once.  Rustic way of doing it but in the given time
    tic; % Start timer
    while toc < travelTime*2

        %If 1st third of timer go to 2nd corner
        if toc < travelTime/3 && counter2 == 1
            disp("Pos1");
            robot.interpolate_jp(point2, travelTime*1000/3);
            counter2 = counter2 + 1;
        %If 2nd third of timer go to 3rd corner
        elseif toc > travelTime/3 && toc < travelTime/3*2 && counter2 == 2 
            disp("Pos2");
            robot.interpolate_jp(point3, travelTime*1000/3);
            counter2 = counter2 + 1;
        %If 3rd third of timer go back to first coerner
        elseif toc > travelTime/3*2 && counter2 == 3
            disp("Pos3");
            robot.interpolate_jp(point1, travelTime*1000/3);  
        end

        %%Data Recording
        EE = robot.fk3001(robot.setpoint_js);  %Create the transformation matrix
        EEPos = EE(1:3, 4);                    %Trim data for only X, Y, Z locations
        EEPos = EEPos';                        %Transpose it for easier csv file record
        timer = toc;
        A = [timer, robot.setpoint_js, EEPos];        %Combine setpoint along with current point so C1 = Theta1 C2 = Theta2, C3 = Theta 3, C4 = Theta 4, C5 = X, C6 = Y, C7 = Z
        jointPosition(counter, :) = A;         %Put into major matrix for one csv file
        counter = counter + 1;                 %Increment CSV file matrix counter
    end

     writematrix(jointPosition, 'Triangle.csv');  %Push matrix into CSV
     jointPosition = readmatrix('Triangle.csv');
    
     
     Ya1 = jointPosition(1:625,3); %Joint angle 1
     Ya2 = jointPosition(1:625,4); %Joint angle 2
     Ya3 = jointPosition(1:625,5); %Joint angle 3
     Ya4 = jointPosition(1:625,6); %X Value of End Effector
     Ya5 = jointPosition(1:625,7); %Y Value of End Effector
     Ya6 = jointPosition(1:625,8); %Z Value of End Effector
     X = jointPosition(1:625,1); %Time
     
     figure(1);
     plot(X, Ya1, 'LineWidth', 3, 'DisplayName', 'Joint 1');
     hold on
     plot(X, Ya2, 'LineWidth', 3, 'DisplayName', 'Joint 2');
     plot(X, Ya3, 'LineWidth', 3, 'DisplayName', 'Joint 3');
     hold off
     grid
     xlabel('Time (seconds)')
     ylabel('Joint Angles (degrees)')
     legend

     figure(2);
     plot(X, Ya4, 'LineWidth', 3, 'DisplayName', 'X Value');
     hold on
     plot(X, Ya6, 'LineWidth', 3, 'DisplayName', 'Z Value');
     hold off
     grid
     xlabel('Time (seconds)')
     ylabel('Location of end effector (mm)')
     legend
      
     figure(3);
     scatter(Ya4, Ya6, 'DisplayName', 'XZ Plane Movement');
     hold on
     scatter(282, 214, '^', 'filled', 'DisplayName', 'vertice');
     scatter(87, 440, '^', 'filled', 'DisplayName', 'vertice');
     scatter(83, 394.4, '^', 'filled', 'DisplayName', 'vertice');
     hold off
     axis([0 350 150 500]);
     
     
     figure(4);
     scatter(Ya4, Ya5, 'DisplayName', 'XY Plane Movement');
     hold on
     scatter(282, .43, '^', 'filled', 'DisplayName', 'vertice');
     scatter(87, .13, '^', 'filled', 'DisplayName', 'vertice');
     scatter(83, .12, '^', 'filled', 'DisplayName', 'vertice');
     hold off
     axis([0 500 -50 50]);

     disp("Done");
     