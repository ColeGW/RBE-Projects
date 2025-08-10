%% Setup robot
close all
travelTime = 10; % Defines the travel time
robot = Robot(); % Creates robot object
model = Model(); % Creates model object
%robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
%% Program 
robot.writeJoints(0);
point1 = [300, 150, 200, 0];      %First Corner of Triangle
point2 = [150, 0, 320, 0];  %Second Corner of Triangle
point3 = [200, 50, 150, 0];    %Third Corner of Triange

robot.writeJoints(robot.ik3001(point1)); % Go to 1st corner

pause(travelTime/2); % Wait for trajectory completion



jointPosition = zeros(10000, 8);

    counter = 1;   %counter is to iterate for the CSV table to put data into a matrix
    counter2 = 1;  %counter2 is to iterate through robot movements so that the function is only ever called once.  Rustic way of doing it but in the given time
    tic; % Start timer
    while toc < travelTime*1.5

        %If 1st third of timer go to 2nd corner
        if toc < travelTime/3 && counter2 == 1
            %disp("Pos2");
            robot.interpolate_jp(robot.ik3001(point2), travelTime*1000/3);
            counter2 = counter2 + 1;
        %If 2nd third of timer go to 3rd corner
        elseif toc > travelTime/3 && toc < travelTime*2/3 && counter2 == 2 
            %disp("Pos3");
            robot.interpolate_jp(robot.ik3001(point3), travelTime*1000/3);
            counter2 = counter2 + 1;
        %If 3rd third of timer go back to first coerner
        elseif toc > travelTime*2/3 && toc < travelTime && counter2 == 3
            disp("Pos1");
            robot.interpolate_jp(robot.ik3001(point1), travelTime*1000/3); 
%         elseif toc > travelTime && counter2 == 3
%             disp("Return to Pos1");
%             robot.interpolate_jp(robot.ik3001(point3), travelTime*1000/3);
        end

        model.plot_arm(robot.setpoint_js()); % Read joint values into model plot so that it live plots
        %%Data Recording
        EE = robot.fk3001(robot.setpoint_js);  %Create the transformation matrix
        EEPos = EE(1:3, 4);                    %Trim data for only X, Y, Z locations
        EEPos = EEPos';                        %Transpose it for easier csv file record
        timer = toc;
        A = [timer, robot.setpoint_js, EEPos];        %Combine setpoint along with current point so C1 = Theta1 C2 = Theta2, C3 = Theta 3, C4 = Theta 4, C5 = X, C6 = Y, C7 = Z
        jointPosition(counter, :) = A;         %Put into major matrix for one csv file
        counter = counter + 1;                 %Increment CSV file matrix counter
        velocities = robot.measured_js(0, 1);
        disp(velocities(2, :))
    end
     writematrix(jointPosition, 'Triangle3.csv');  %Push matrix into CSV
     cutoff = 180;
     Ya0 = jointPosition(1:cutoff,2); %Base joint angle 1
     Ya1 = jointPosition(1:cutoff,3); %Joint angle 1
     Ya2 = jointPosition(1:cutoff,4); %Joint angle 2
     Ya3 = jointPosition(1:cutoff,5); %Joint angle 3
     Ya4 = jointPosition(1:cutoff,6); %X Value of End Effector
     Ya5 = jointPosition(1:cutoff,7); %Y Value of End Effector
     Ya6 = jointPosition(1:cutoff,8); %Z Value of End Effector
     X = jointPosition(1:cutoff,1); %Time

     % Line plot for XYZ values of end effector
     figure(2);
     plot(X, Ya4, 'LineWidth', 3, 'DisplayName', 'X Value');
     hold on
     plot(X, Ya5, 'LineWidth', 3, 'DisplayName', 'Y Value');
     plot(X, Ya6, 'LineWidth', 3, 'DisplayName', 'Z Value');
     hold off
     grid
     title('End effector XYZ values over time')
     xlabel('Time (seconds)')
     ylabel('Location of end effector (mm)')
     legend
     
     % 3D scatter plot for XYZ values of end effector
     figure(3);
     scatter3(Ya4, Ya5, Ya6, 'd', 'DisplayName', 'XYZ Plane Movement');
     hold on
     scatter3(300, 150, 200, '^', 'DisplayName', 'vertice');
     scatter3(150, 0, 320, '^','DisplayName', 'vertice');
     scatter3(200, 50, 150, '^', 'DisplayName', 'vertice');
     title('End effector XYZ values')
     xlabel('X Value (mm)')
     ylabel('Y Value (mm)')
     zlabel('Z Value (mm)')
     %writematrix([Ya4, Ya5, Ya6], 'noInterp.csv');
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

     disp("Done");