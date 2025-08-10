syms theta1 theta2 theta3 theta4
 sympref('AbbreviateOutput', false);
%                mDim = [36.076, 60.25, 130.23, 124, 133.4]; % (mm)
%            mOtherDim = [128, 24]; % (mm) 
L0 = 36.076;
L1 = 60.25;
L2 = 130.23;
L3 = 124; 
L4 = 133.4;
        [T1, T2, T3, T4, T5] = fk3001([theta1, theta2, theta3, theta4], [L0 L1 L2 L3 L4]);
        AB = T5;
        X = AB(1, 4);
        Y = AB(2, 4);
        Z = AB(3, 4);
        

        Jp1 = [diff(X, theta1), diff(Y, theta1), diff(Z, theta1)]';
        Jp2 = [diff(X, theta2), diff(Y, theta2), diff(Z, theta2)]';
        Jp3 = [diff(X, theta3), diff(Y, theta3), diff(Z, theta3)]';
        Jp4 = [diff(X, theta4), diff(Y, theta4), diff(Z, theta4)]';
        Jo1 = [T1(1:3, 3)];
        Jo2 = [T2(1:3, 3)];
        Jo3 = [T3(1:3, 3)];
        Jo4 = [T4(1:3, 3)];
        
        FirstRow = [Jp1(1), Jp2(1), Jp3(1), Jp4(1)];
        SecondRow = [Jp1(2), Jp2(2), Jp3(2), Jp4(2)];
        ThirdRow = [Jp1(3), Jp2(3), Jp3(3), Jp4(3)];
        FourthRow = [Jo1(1), Jo2(1), Jo3(1), Jo4(1)];
        FifthRow = [Jo1(2), Jo2(2), Jo3(2), Jo4(2)];
        SixthRow = [Jo1(3), Jo2(3), Jo3(3), Jo4(3)];
        disp(FirstRow);
        disp(SecondRow);
        disp(ThirdRow);
        disp(FourthRow);
        disp(FifthRow);
        disp(SixthRow);
        JpCombined = [Jp1, Jp2, Jp3, Jp4; Jo1, Jo2, Jo3, Jo4];
        save('SymbolicJacobian.mat', 'JpCombined');
        %Generates homogeneous transformation matrix
        %Takes in a 1x4 array - then calculates the 4x4 homogeneous
        %transformation matrix

%%
        function T = dh2mat(dhtable1)
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
        function [T1, T2, T3, T4, T5] = dh2fk(dhtable)
            n=5;
            T = eye(4);
            for i = 1:n
                placeHolder = dh2mat(dhtable(i,:));
                T = T * placeHolder;  %%Correct matrix mutliplication
                if(i == 1)
                    T1 = T;
                elseif(i == 2)
                    T2 = T;
                elseif(i == 3)
                    T3 = T;
                elseif(i == 4)
                    T4 = T;
                else
                    T5 = T;
                end
            end
        end
        
        %Returns foreward kinematic transformation table
        %Takes in n joint values as inputs in the form of a nx1 vector and
        %returns a 4x4 homogeneous transformation matrix representing the
        %position and orientation of the tip frame
        function [T1, T2, T3, T4, T5] = fk3001(angleTable, dimTable)
%            mDim = [36.076, 60.25, 130.23, 124, 133.4]; % (mm)
            mOtherDim = [128, 24]; % (mm)   
          
           zTheta12 = -90;
           DHTable = sym(zeros(5, 4));
           DHTable(1, :) = [0, dimTable(1), 0, 0];
           DHTable(2, :) = [angleTable(1), dimTable(2), 0, zTheta12];
           DHTable(3, :) = [(angleTable(2) - atand(mOtherDim(1)/mOtherDim(2))), 0, dimTable(3), 0];
           DHTable(4, :) = [(angleTable(3) + atand(mOtherDim(1)/mOtherDim(2))), 0, dimTable(4), 0];
           DHTable(5, :) = [angleTable(4), 0, dimTable(5), 0];
           [T1, T2, T3, T4, T5] = dh2fk(DHTable);

        end