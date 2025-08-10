% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Model
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Model()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            set(gcf,'Visible','on');
            
        end

        
        %Generates homogeneous transformation matrix
        %Takes in a 1x4 array - then calculates the 4x4 homogeneous
        %transformation matrix
        function T = dh2mat(self, dhtable1)
            theta = dhtable1(1);
            d = dhtable1(2);
            a = dhtable1(3);
            alpha = dhtable1(4);
            T = [cosd(theta), -sind(theta)*cosd(alpha), sind(theta)*sind(alpha), a*cosd(theta);
            sind(theta), cosd(theta)*cosd(alpha), -cosd(theta)*sind(alpha), a*sind(theta);
            0, sind(alpha), cosd(alpha), d;
            0, 0, 0, 1];
        end
        

        %Generates symbolic homogeneous transformation matrix
        %takes in an nx4 corresponding to the n rows of the full DH
        %parameter table
        function T = dh2fk(self, dhtable)
            %n=5
            %{
            T1 = dh2mat(self, dhtable(1, :));
            T2 = dh2mat(self, dhtable(2, :));
            T3 = dh2mat(self, dhtable(3, :));
            T4 = dh2mat(self, dhtable(4, :));
            T5 = dh2mat(self, dhtable(5, :));
            %}
            T = eye(4);
            for i = 1:5
                placeHolder = self.dh2mat(dhtable(i,:));
                T = T * placeHolder;
            end
        end

        %Returns foreward kinematic transformation table
        %Takes in n joint values as inputs in the form of a nx1 vector and
        %returns a 4x4 homogeneous transformation matrix representing the
        %position and orientation of the tip frame
        function T = fk3001(self, angleTable)
           
           zTheta12 = -90;
           DHTable = zeros(5, 4);
           DHTable(1, :) = [0, self.mDim(1), 0, 0];
           DHTable(2, :) = [angleTable(1), self.mDim(2), 0, zTheta12];
           DHTable(3, :) = [(angleTable(2) - atand(self.mOtherDim(1)/self.mOtherDim(2))), 0, self.mDim(3), 0];
           DHTable(4, :) = [(angleTable(3) + atand(self.mOtherDim(1)/self.mOtherDim(2))), 0 self.mDim(4), 0];
           DHTable(5, :) = [angleTable(4), 0, self.mDim(5), 0];
           T = self.dh2fk(self, DHTable);
        end        
        

        function plot_arm(self, fourangles)%, velocities)
            
            shg %For update plot
            %disp(velocities') For velocities
           
            xCord = [0 self.getValue(1, 4, fourangles)];
            yCord = [0 self.getValue(2, 4, fourangles)];
            zCord = [0 self.getValue(3, 4, fourangles)];
%             XV = velocities(1, 1);
%             YV = velocities(2, 1);
%             ZV = velocities(3, 1);
            plot3(xCord, yCord, zCord, '.-', 'LineWidth', 2, 'MarkerSize', 20);
            hold on
            for i = 1:3
                x01Array = self.getValue(1, i, fourangles); %X0 for X1,Y1,Z1
                y01Array = self.getValue(2, i, fourangles); %Y0 or X1,Y1,Z1
                z01Array = self.getValue(3, i, fourangles); %Z0 for X1,Y1,Z1
                quiver3(xCord, yCord, zCord, [0 x01Array], [0 y01Array], [0 z01Array]);
            end
            %quiver3(xCord(6), yCord(6), zCord(6) , XV, YV, ZV, 'r', 'LineWidth', 2);
            grid on
            grid minor
            set(gca,'GridLineStyle','--')
            title("Live Plot with XYZ frames per frame");
            xlabel("X-Axis (mm)");
            ylabel("Y-Axis (mm)");
            zlabel("Z-Axis (mm)");

            axis([-400 400 -400 400 0 500]);
            drawnow;
            hold off
           
        end

        function T = getValue(self,row, column, fourangles)
            mDim = [36.076, 60.25, 130.23, 124, 133.4]; % (mm)
            mOtherDim = [128, 24]; % (mm)
            zTheta12 = -90;
            T1 = self.dh2mat([0, mDim(1), 0, 0]);
            T2 = T1 * self.dh2mat([fourangles(1), mDim(2), 0, zTheta12]);
            T3 = T2 * self.dh2mat([(fourangles(2) - atand(mOtherDim(1)/mOtherDim(2))), 0, mDim(3), 0]);
            T4 = T3 * self.dh2mat([(fourangles(3) + atand(mOtherDim(1)/mOtherDim(2))), 0 mDim(4), 0]);
            T5 = T4 * self.dh2mat([fourangles(4), 0, mDim(5), 0]);
            T = [T1(row, column), T2(row, column), T3(row, column),T4(row, column), T5(row, column)];
        end

         function plot_armVel(self, fourangles, velocities)
            
          %  shg %For update plot
            %disp(velocities') %For velocities
           
            xCord = [0 self.getValue(1, 4, fourangles)];
            yCord = [0 self.getValue(2, 4, fourangles)];
            zCord = [0 self.getValue(3, 4, fourangles)];
            XV = velocities(1, 1);
            YV = velocities(2, 1);
            ZV = velocities(3, 1);
            plot3(xCord, yCord, zCord, '.-', 'LineWidth', 2, 'MarkerSize', 20);
            hold on
            for i = 1:3
                x01Array = self.getValue(1, i, fourangles); %X0 for X1,Y1,Z1
                y01Array = self.getValue(2, i, fourangles); %Y0 or X1,Y1,Z1
                z01Array = self.getValue(3, i, fourangles); %Z0 for X1,Y1,Z1
                quiver3(xCord, yCord, zCord, [0 x01Array], [0 y01Array], [0 z01Array]);
            end
            quiver3(xCord(6), yCord(6), zCord(6) , XV, YV, ZV, 'r', 'LineWidth', 2);
            grid on
            grid minor
            set(gca,'GridLineStyle','--')
            title("Live Plot with XYZ frames per frame");
            xlabel("X-Axis (mm)");
            ylabel("Y-Axis (mm)");
            zlabel("Z-Axis (mm)");

            axis([-400 400 -400 400 0 500]);
            drawnow;
            hold off
           
        end

        
    end % end methods
end % end class 
