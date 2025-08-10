% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        mDHTable;

    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);
            self.mDim = [36.076, 60.25, 130.23, 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            % Robot Dimensions
      
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
            self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
        end
        
        % Sends the joints to the desired angles without interpolation
        % goals [1x4 double] - angles (degrees) for each of the joints to
        % go to
        function servo_jp(self, goals)
%             self.writeTime(0.02);
            self.writeJoints(goals);
        end
        
        %Takes 1x4 array of joint values and an interpolation time in ms to
        %get there
        %goals [1x4 double] - angles of joint values
        %time time - time in ms to get there
        function interpolate_jp(self, goals, time)
            self.writeTime(time/1000);
            self.writeJoints(goals);
        end

        % Returns a 1x4 array that contains 
        % current joint set point positions in degrees
        % (deg)
        function readings = setpoint_js(self)
            readings = zeros(1,4);
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
        end

        % Which takes  two  boolean  values, named  GETPOS  and GETVEL. Only  
        % return the  results  for the requested data, and set the rest to
        % zero.
        % Which returns a 2x4 array that contains current joint positions in degrees (1st row) and/or 
        % current joint velocities (2nd row).
	    function out = measured_js(self, GETPOS, GETVEL)
            out = zeros(2,4);  % Initalize matrix to zeros
            if GETPOS   % If we want the position
                out(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            end 
            
            if GETVEL   % If we want the velocity
                out(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            end
        end

        %goal_js()
        function endofmotion = goal_js(self)
            endofmotion = zeros(1,4);
            endofmotion(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
        
        end

        

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
      
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;
% 
%             disp("time")
%             disp(time_ms)
%             disp("acc time")
%             disp(acc_time_ms)

            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end
     
        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the dispdesired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end
        

        %Generates homogeneous transformation matrix
        %Takes in a 1x4 array - then calculates the 4x4 homogeneous
        %transformation matrix
        function T = dh2mat(~, dhtable1)
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
        %Times each transformation matrix to get Frame 0 to Frame EE
        function T = dh2fk(self, dhtable)
            n=5;
            T = eye(4);
            for i = 1:n
                placeHolder = self.dh2mat(dhtable(i,:));
                T = T * placeHolder;  %%Correct matrix mutliplication
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
           T = self.dh2fk(DHTable);

        end
        
        %Calculates the 4x4 homogeneous transformation matrix 
        % based on the current measured joint positions
        function T = measured_cp(self)
            m_angs = self.measured_js(1, 0);
            angs = zeros(4,1);
            angs(1, 1) = m_angs(1, 1);
            angs(2, 1) = m_angs(1, 2);
            angs(3, 1) = m_angs(1, 3);
            angs(4, 1) = m_angs(1, 4);
            T = self.fk3001(angs);
        end
        
        %Calculates the 4x4 homogeneous transformation matrix 
        % based on the current measured joint set positions
        function T = setpoint_cp(self)
            s_angs = self.setpoint_js();
            angs = zeros(4,1);
            angs(1, 1) = s_angs(1, 1);
            angs(2, 1) = s_angs(1, 2);
            angs(3, 1) = s_angs(1, 3);
            angs(4, 1) = s_angs(1, 4);
            T = self.fk3001(angs);
        end

        %Calculates the 4x4 homogeneous transformation matrix 
        % based on the current end motion joint set positions
        function T = goal_cp(self)
            g_angs = self.goal_js();
            angs = zeros(4,1);
            angs(1, 1) = g_angs(1, 1);
            angs(2, 1) = g_angs(1, 2);
            angs(3, 1) = g_angs(1, 3);
            angs(4, 1) = g_angs(1, 4);
            T = self.fk3001(angs);
        end

        %Plot function for Lab 2 #4
        %Takes our ideal calcaulated value for the joint positions and
        %creates a plot with the actual, average and root mean square.
        function plott(self, T1, T2, T3, T4, T5, Tactual)
                % This represnts the robots actual values moving from the
                % Target and home position
                x1 = [T1(1,4)];
                y1 = [T1(2,4)];
                z1 = [T1(3,4)];
                x2 = [T2(1,4)];
                y2 = [T2(2,4)];
                z2 = [T2(3,4)];
                x3 = [T3(1,4)];
                y3 = [T3(2,4)];
                z3 = [T3(3,4)];
                x4 = [T4(1,4)];
                y4 = [T4(2,4)];
                z4 = [T4(3,4)];
                x5 = [T5(1,4)];
                y5 = [T5(2,4)];
                z5 = [T5(3,4)];
                scatter3(x1,y1,z1,'filled','Color','r','DisplayName','T1')
                hold on
                scatter3(x2,y2,z2,'filled','Color','g','DisplayName','T2')
                scatter3(x3,y3,z3,'filled','Color','b','DisplayName','T3')
                scatter3(x4,y4,z4,'filled','Color','c','DisplayName','T4')
                scatter3(x5,y5,z5,'filled','Color','m','DisplayName','T5')
                % That actual value that is calculated from our FK equation
                xt = Tactual(1,4);
                yt = Tactual(2,4);
                zt = Tactual(3,4);
                scatter3(xt,yt,zt,'filled','Color','y','DisplayName','Tactual')
                % Averages out the data set
                xaverage = ((T1(1,4) + T2(1,4) + T3(1,4) + T4(1,4) + T5(1,4))/5);
                yaverage = ((T1(2,4) + T2(2,4) + T3(2,4) + T4(2,4) + T5(2,4))/5);
                zaverage = ((T1(3,4) + T2(3,4) + T3(3,4) + T4(3,4) + T5(3,4))/5);
                scatter3(xaverage,yaverage,zaverage,'filled','Color','k','DisplayName','Average T1-T5')
                xlabel('X');
                ylabel('Y');
                zlabel('Z');
                xlim([280 290]);
                ylim([0 1]);
                zlim([200 230]);
                legend
                grid on
                grid minor
                hold off
        end

        % Calculates the RMSE from the data collected and the actual data
        %Takes in calculated data and data collected and returns Root Mean
        %Squared
        function T = rmse(self, data, actual)
            T = sqrt(sum((data(:)-actual(:)).^2) / numel(data));
        end
        
        %Calculates the Inverse Kinematics 
        %Takes in the XYZ of the end affector position as well as alpha
        %which is the claws orientation with respect to the floor parallel
        %= 0 pointed towards the ground = 90 degrees
        %Using the geometric approach which can be found in our lab report
        %with more explanantion
        % Returns the angle the motors should be at for a specific EE and
        % Alpha
        function T = ik3001(self, jsv)
            %mm and degrees
            %Givens:
            Xc = jsv(1);
            Yc = jsv(2);
            Zc = jsv(3);
            alpha = jsv(4);
            r = sqrt(Xc^2 + Yc^2);
            lzero = self.mDim(1);
            lone = self.mDim(2);
            ltwo = self.mDim(3);
            lthree = self.mDim(4);
            lfour = self.mDim(5);

            
            %Start Basic Calculations / predetermined stuff
            hone = lfour*sind(alpha);  %H1
            zfour = Zc + hone; %Z4
            rtwo = cosd(alpha)*lfour; %r2
            ztwo = lone + lzero;
            xtwo = 0;
            ytwo = 0;
            preAngle = atan2d(sqrt(1-(self.mOtherDim(1)/ltwo)^2), self.mOtherDim(1)/ltwo);
            %Theta Calculations:  Theta1 = q1, etc. in degrees

            %Q1 Calc
            q1 = atan2d(sqrt(1-(Xc/r)^2), Xc/r);
            q1other = atan2d(-sqrt(1-(Xc/r)^2), Xc/r);
            
%             disp("test");
%             disp(q1other);
            if(alpha < 0)
                q1holder = q1;
                q1 = q1other;
                q1other = q1holder;
            end 
            if(q1 >= 100 || q1 <= -100)
                q1 = q1other;
            end
            if(q1 > 0 && Yc < 0)
                q1 = q1other;
            end
            if(q1 >= 100 || q1 <= -100)
                error("Not Possible");
            end
            
            done = cosd(q1) * rtwo; %d1
            wone = sind(q1) * rtwo; %w1
            xfour = Xc - done;%x4
            yfour = Yc - wone; %y4
            xy = sqrt((xfour - xtwo)^2 + (yfour - ytwo)^2); %xy line for three dimension triangle
            rthree = sqrt((xy)^2+(zfour-ztwo)^2);


            %Q2 Calc
            
            D = (lthree^2 - rthree^2 - ltwo^2) / (-2*rthree*ltwo);
            if(D > 1)
                D = 1; %%Small Error in Calc caused by roundoff
            end
            Beta = atan2d(sqrt(1-D^2), D);
            Betaother = atan2d(-sqrt(1-D^2), D);
            
            phiHelper = (zfour - ztwo)/rthree;
            phi = atan2d(sqrt(1-phiHelper^2), phiHelper);
            phiother = atan2d(-sqrt(1-phiHelper^2), phiHelper);

            q2 = phi - Beta; %Theta2 = phi - Beta
            q2other = phi - Betaother;
            q2third = phiother - Betaother;
            q2fourth = phiother - Beta;
            if(q2 > 95 || q2 < -95)
                q2 = q2other;
            end       
            if(q2 > 95 || q2 < -95)
                q2 = q2third;
            end
            if(q2 > 95 || q2 < -95)
                q2 = q2fourth;
            end
            if(q2 > 95 || q2 < -95)
                error("Not Possible");
            end
            q2 = q2 - preAngle; %Subtract initial offset angle


            %Q3 Calc

            htwo = cosd(q2 + preAngle) * ltwo; %need to account for right trianlge between m2 and m3
            zthree = ztwo + htwo;
            hthree = zthree - zfour;
            if(abs(hthree) > lthree)

                hthree = lthree;
            end
                q3 = atan2d(hthree/lthree, sqrt(1-(hthree/lthree)^2));
                q3other = atan2d(hthree/lthree, -sqrt(1-(hthree/lthree)^2));
            if(q3 >= 90 || q3 <=-90)
                q3 = q3other;
            end
            if(q3 >= 95 || q3 <= -95)
                error("Not Possible");
            end 
            q3 = q3 - q2;

            %Q4 Calc
            q4 = alpha - q3 - q2;

            T = [q1, q2, q3, q4];
        end
        

        %Run_trajectory calculates quintic and cubic either task or Joint
        %Space Trajectory
        %It takes in the the Coeff which is a 4x4 or a 6x4 based on quintic
        %or cubic and then takes in the time that the trajectory planner
        %was calculated on.  Then takes a 1 or 0 for task to determine if
        %it is task or joint space        %
        %Returns an array of the jointPosition
        function T = run_trajectory(self, coef, time, task)%, model)  %put model here for velocity as an input for signoff 2
            S = size(coef);
            runcounter = 1;
            jointPosition = zeros(10000, 5);
            estop = 1;
                tic; % Start timer
                    while (toc < time) && estop
                        if (task == 1) % Is task space
                            if (S(1) == 4) % Is cubic
                                q0 = coef(1,1) + coef(2,1) * toc + coef(3,1) * toc^2 + coef(4,1)*toc^3;
                                q1 = coef(1,2) + coef(2,2) * toc + coef(3,2) * toc^2 + coef(4,2)*toc^3;
                                q2 = coef(1,3) + coef(2,3) * toc + coef(3,3) * toc^2 + coef(4,3)*toc^3;
                                q3 = coef(1,4) + coef(2,4) * toc + coef(3,4) * toc^2 + coef(4,4)*toc^3;
                                Q = self.ik3001([q0, q1, q2, q3]);
                                q0 = Q(1,1);
                                q1 = Q(1,2);
                                q2 = Q(1,3);
                                q3 = Q(1,4);
                                self.servo_jp([q0, q1, q2, q3]);
                                estop = self.findDet([q0, q1, q2, q3]);
                                jointPosition(runcounter, :) = [toc, q0, q1, q2, q3];
                            elseif (S(1) == 6) % Is quintic
                               
                                q0 = coef(1,1) + coef(2,1) * toc + coef(3,1) * toc^2 + coef(4,1)*toc^3 + coef(5,1)*toc^4 + coef(6,1)*toc^5;
                                q1 = coef(1,2) + coef(2,2) * toc + coef(3,2) * toc^2 + coef(4,2)*toc^3 + coef(5,2)*toc^4 + coef(6,2)*toc^5;
                                q2 = coef(1,3) + coef(2,3) * toc + coef(3,3) * toc^2 + coef(4,3)*toc^3 + coef(5,3)*toc^4 + coef(6,3)*toc^5;
                                q3 = coef(1,4) + coef(2,4) * toc + coef(3,4) * toc^2 + coef(4,4)*toc^3 + coef(5,4)*toc^4 + coef(6,4)*toc^5;
                                Q = self.ik3001([q0, q1, q2, q3]);
                                q0 = Q(1,1);
                                q1 = Q(1,2);
                                q2 = Q(1,3);
                                q3 = Q(1,4);
                                estop = self.findDet([q0, q1, q2, q3]);
                                self.servo_jp([q0, q1, q2, q3]);
                                jointPosition(runcounter, :) = [toc, q0, q1, q2, q3];
                            end
                        elseif (task == 0) % Is joint space
                            if (S(1) == 4) % Is cubic
                                q0 = coef(1,1) + coef(2,1) * toc + coef(3,1) * toc^2 + coef(4,1)*toc^3;
                                q1 = coef(1,2) + coef(2,2) * toc + coef(3,2) * toc^2 + coef(4,2)*toc^3;
                                q2 = coef(1,3) + coef(2,3) * toc + coef(3,3) * toc^2 + coef(4,3)*toc^3;
                                q3 = coef(1,4) + coef(2,4) * toc + coef(3,4) * toc^2 + coef(4,4)*toc^3;
                                estop = self.findDet([q0, q1, q2, q3]);
                                self.servo_jp([q0, q1, q2, q3]);
                                jointPosition(runcounter, :) = [toc, q0, q1, q2, q3];
                            elseif (S(1) == 6) % Is quintic
                                q0 = coef(1,1) + coef(2,1) * toc + coef(3,1) * toc^2 + coef(4,1)*toc^3 + coef(5,1)*toc^4 + coef(6,1)*toc^5;
                                q1 = coef(1,2) + coef(2,2) * toc + coef(3,2) * toc^2 + coef(4,2)*toc^3 + coef(5,2)*toc^4 + coef(6,2)*toc^5;
                                q2 = coef(1,3) + coef(2,3) * toc + coef(3,3) * toc^2 + coef(4,3)*toc^3 + coef(5,3)*toc^4 + coef(6,3)*toc^5;
                                q3 = coef(1,4) + coef(2,4) * toc + coef(3,4) * toc^2 + coef(4,4)*toc^3 + coef(5,4)*toc^4 + coef(6,4)*toc^5;
                               
                                estop = self.findDet([q0, q1, q2, q3]);
%                                 jacob = self.jacob3001([q0, q1, q2, q3]);
%                                 detrm = det(jacob(1:3, 1:3));
%                                 self.servo_jp([q0, q1, q2, q3]);  //FOR
%                                 PLOTTING SERVICES

                                jointPosition(runcounter, :) = [toc, q0, q1, q2, q3,]; %Plotting Services detrm];
                            
                        end
                        runcounter = runcounter + 1;
                      
%                         pause(0.1); %0.001
                        end
                         pause(0.001); %0.001
                    end
                T = jointPosition;
        end
        
        %
        %Run_trajectory Plot is the same as run_trajectory execept it live plots which takes a while so we made it 
        % two seperate functions
        % calculates quintic and cubic either task or Joint
        %Space Trajectory
        %It takes in the the Coeff which is a 4x4 or a 6x4 based on quintic
        %or cubic and then takes in the time that the trajectory planner
        %was calculated on.  Then takes a 1 or 0 for task to determine if
        %it is task or joint space
        %It also takes in the model to plot
        %Finally it takes in count which determines which points/matrix to
        %write into
        %Returns an array of the jointPosition
        function T = run_trajectoryPLOT(self, coef, time, task, model, count)  %put model here for velocity as an input for signoff 2
            S = size(coef);
            runcounter = 1;
            
            counter = count;
            jointPosition = zeros(10000, 5);
            jacobSpeeds = zeros(105, 7);
                tic; % Start timer
                    while toc < time
                        disp(runcounter);
                        outputs = self.getJointsReadings();
                        outputs(2, 2) = outputs(2,2) - 61.83;
                        outputs(2, 3) = outputs(2, 3) - 65.952;
                        A = self.fdk3001(outputs(1, :), outputs(2, :));
                        A = A';
                        jacobSpeeds(counter, :) = [A toc];         %Put into major matrix for one csv file
                        counter = counter + 1;
                        %model.plot_armVel(outputs(1,:), self.fdk3001(outputs(1,:), outputs(2, :)));  %Comment out to reduce Jittery
                        %pause(0.001);  %Pause if no live plot
                        if (task == 1) % Is work space
                            if (S(1) == 4) % Is cubic
                                q0 = coef(1,1) + coef(2,1) * toc + coef(3,1) * toc^2 + coef(4,1)*toc^3;
                                q1 = coef(1,2) + coef(2,2) * toc + coef(3,2) * toc^2 + coef(4,2)*toc^3;
                                q2 = coef(1,3) + coef(2,3) * toc + coef(3,3) * toc^2 + coef(4,3)*toc^3;
                                q3 = coef(1,4) + coef(2,4) * toc + coef(3,4) * toc^2 + coef(4,4)*toc^3;
                                Q = self.ik3001([q0, q1, q2, q3]);
                                q0 = Q(1,1);
                                q1 = Q(1,2);
                                q2 = Q(1,3);
                                q3 = Q(1,4);
                                self.servo_jp([q0, q1, q2, q3]);
                                jointPosition(runcounter, :) = [toc, q0, q1, q2, q3];
                            elseif (S(1) == 6) % Is quintic
                               
                                q0 = coef(1,1) + coef(2,1) * toc + coef(3,1) * toc^2 + coef(4,1)*toc^3 + coef(5,1)*toc^4 + coef(6,1)*toc^5;
                                q1 = coef(1,2) + coef(2,2) * toc + coef(3,2) * toc^2 + coef(4,2)*toc^3 + coef(5,2)*toc^4 + coef(6,2)*toc^5;
                                q2 = coef(1,3) + coef(2,3) * toc + coef(3,3) * toc^2 + coef(4,3)*toc^3 + coef(5,3)*toc^4 + coef(6,3)*toc^5;
                                q3 = coef(1,4) + coef(2,4) * toc + coef(3,4) * toc^2 + coef(4,4)*toc^3 + coef(5,4)*toc^4 + coef(6,4)*toc^5;
                                Q = self.ik3001([q0, q1, q2, q3]);
                                q0 = Q(1,1);
                                q1 = Q(1,2);
                                q2 = Q(1,3);
                                q3 = Q(1,4);
                                self.servo_jp([q0, q1, q2, q3]);
                                jointPosition(runcounter, :) = [toc, q0, q1, q2, q3];
                            end
                        elseif (task == 0) % Is task space
                            if (S(1) == 4) % Is cubic
                                q0 = coef(1,1) + coef(2,1) * toc + coef(3,1) * toc^2 + coef(4,1)*toc^3;
                                q1 = coef(1,2) + coef(2,2) * toc + coef(3,2) * toc^2 + coef(4,2)*toc^3;
                                q2 = coef(1,3) + coef(2,3) * toc + coef(3,3) * toc^2 + coef(4,3)*toc^3;
                                q3 = coef(1,4) + coef(2,4) * toc + coef(3,4) * toc^2 + coef(4,4)*toc^3;
                                self.servo_jp([q0, q1, q2, q3]);
                                jointPosition(runcounter, :) = [toc, q0, q1, q2, q3];
                            elseif (S(1) == 6) % Is quintic
                                q0 = coef(1,1) + coef(2,1) * toc + coef(3,1) * toc^2 + coef(4,1)*toc^3 + coef(5,1)*toc^4 + coef(6,1)*toc^5;
                                q1 = coef(1,2) + coef(2,2) * toc + coef(3,2) * toc^2 + coef(4,2)*toc^3 + coef(5,2)*toc^4 + coef(6,2)*toc^5;
                                q2 = coef(1,3) + coef(2,3) * toc + coef(3,3) * toc^2 + coef(4,3)*toc^3 + coef(5,3)*toc^4 + coef(6,3)*toc^5;
                                q3 = coef(1,4) + coef(2,4) * toc + coef(3,4) * toc^2 + coef(4,4)*toc^3 + coef(5,4)*toc^4 + coef(6,4)*toc^5;
                                self.servo_jp([q0, q1, q2, q3]);
                                
                                jointPosition(runcounter, :) = [toc, q0, q1, q2, q3, ];
                         
                                
                        end
                        runcounter = runcounter + 1;
                      
                        pause(0.0001); %0.001
                        end
                    end
               
                    if count == 1
                        writematrix(jacobSpeeds, 'Jacob1.csv');
                    elseif count == 2
                        writematrix(jacobSpeeds, 'Jacob2.csv');
                    else
                        writematrix(jacobSpeeds, 'Jacob3.csv');
                    end
                T = jacobSpeeds;
        end

            
        %Jacob3001 calculates the robotics jacobian
        %It does this my taking in the current joint angles and plugging it
        %into the symbolic equation that we found in our symbolicJacobian
        %file that is a live function (We just copied it over)
        %The currentAngles should be in degrees
        %This outputs a 6x4 Jacobian
        function T = jacob3001(self, currentAngles)
          
            theta1 = currentAngles(1);
            theta2 = currentAngles(2);
            theta3 = currentAngles(3);
            theta4 = currentAngles(4);
            jpCombined = [
                          (667*sin((pi*conj(theta4))/180)*((pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/180 + (pi*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/180))/5 - (667*cos((pi*conj(theta4))/180)*((pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/180 - (pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/180))/5 - (1447*pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/2000 - (31*pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/45 + (31*pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/45, (667*sin((pi*conj(theta4))/180)*((pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/180 - (pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/180))/5 - (667*cos((pi*conj(theta4))/180)*((pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/180 + (pi*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/180))/5 - (1447*pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/2000 - (31*pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/45 - (31*pi*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/45, (667*sin((pi*conj(theta4))/180)*((pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/180 - (pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/180))/5 - (667*cos((pi*conj(theta4))/180)*((pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/180 + (pi*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/180))/5 - (31*pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/45 - (31*pi*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/45, - (667*pi*cos((pi*conj(theta4))/180)*(cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180) + cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180)))/900 - (667*pi*sin((pi*conj(theta4))/180)*(cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180) - sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180)))/900;
                          (1447*pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/2000 - (667*sin((pi*conj(theta4))/180)*((pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/180 + (pi*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/180))/5 - (667*cos((pi*conj(theta4))/180)*((pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/180 - (pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/180))/5 - (31*pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/45 + (31*pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*cos((pi*conj(theta1))/180))/45, - (667*cos((pi*conj(theta4))/180)*((pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/180 + (pi*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/180))/5 - (667*sin((pi*conj(theta4))/180)*((pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/180 - (pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/180))/5 - (1447*pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/2000 - (31*pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/45 - (31*pi*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/45, - (667*cos((pi*conj(theta4))/180)*((pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/180 + (pi*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/180))/5 - (667*sin((pi*conj(theta4))/180)*((pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/180 - (pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/180))/5 - (31*pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/45 - (31*pi*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180))/45, - (667*pi*cos((pi*conj(theta4))/180)*(cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180) + cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180)))/900 - (667*pi*sin((pi*conj(theta4))/180)*(cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180) - sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*conj(theta1))/180)))/900;
                          0, (667*sin((pi*conj(theta4))/180)*((pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180))/180 + (pi*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180))/180))/5 - (667*cos((pi*conj(theta4))/180)*((pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180))/180 - (pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180))/180))/5 - (1447*pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180))/2000 - (31*pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180))/45 + (31*pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180))/45, (667*sin((pi*conj(theta4))/180)*((pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180))/180 + (pi*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180))/180))/5 - (667*cos((pi*conj(theta4))/180)*((pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180))/180 - (pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180))/180))/5 - (31*pi*cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180))/45 + (31*pi*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180))/45, (667*pi*sin((pi*conj(theta4))/180)*(cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180) + cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)))/900 - (667*pi*cos((pi*conj(theta4))/180)*(cos((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*cos((pi*(conj(theta3) + 1396473792651755/17592186044416))/180) - sin((pi*(conj(theta2) - 1396473792651755/17592186044416))/180)*sin((pi*(conj(theta3) + 1396473792651755/17592186044416))/180)))/900;
                          0, -sin((pi*theta1)/180), -sin((pi*theta1)/180), -sin((pi*theta1)/180);
                          0, cos((pi*theta1)/180), cos((pi*theta1)/180), cos((pi*theta1)/180);
                          1, 0, 0, 0;];

            T = jpCombined;
        end

        %fdk3001 calculates the linear and angular velocites of the End
        %Effector
        %It takes in the current Angles as well as the current joint
        %velocities and get the jacobian and multiples.
        %
        %Returns the linear and angular velocities
        function T = fdk3001(self, joints, jointsV)
            J = self.jacob3001(joints);
            T = J * jointsV';
        end

        %FindDet takes in current joint Angles and calculate the jacobian
        %Then it takes the 3x3 which is the XYZ velocity of joint 1 2 and 3
        %and calculates the determinant and renturs a 0 if the determinate
        %is too close to a singularity otherwise a 1
        %
        function T = findDet(self, jointAngles)
            jacob = self.jacob3001(jointAngles);
            detrm = det(jacob(1:3, 1:3));
            %disp(detrm);
            if (detrm < 1)
                disp("ERROR STOPPPPP!!!");
                T = 0;
                return;
            end
            T = 1;
        end

    end % end methods
end % end class 
