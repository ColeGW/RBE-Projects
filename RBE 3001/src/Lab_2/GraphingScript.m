%Graph Script

%Array1 = readmatrix('Lab1/InterpolationTest1.csv');
%Array2 = readmatrix('Lab1/InterpolationTest2.csv');
%Array3 = readmatrix('Lab1/InterpolationTest3.csv');

%Array1 = readmatrix('Lab1/ServoTest1.csv');
%Array2 = readmatrix('Lab1/ServoTest2.csv');
%Array3 = readmatrix('Lab1/ServoTest3.csv');
%{
BaseArray1 = Array1(:, 1:2);
BaseArray2 = Array2(:, 1:2);
BaseArray3 = Array3(:, 1:2);
plot(BaseArray1(:,1), BaseArray1(:,2), 'r');
grid on 
hold on
plot(BaseArray2(:,1), BaseArray2(:,2), 'b');
plot(BaseArray3(:,1), BaseArray3(:,2), 'g');
legend('Test 1', 'Test 2', 'Test 3');
title('Base Plot vs Time');
xlabel('Time(s)');
ylabel('Base Motor Degrees');
hold off
%}

trimmedMatrix = readmatrix('Lab1/Pose1.csv');
trimmedMatrixServo = readmatrix('Lab1/ServoPose1.csv');

plot(trimmedMatrix(:,1), trimmedMatrix(:,2), trimmedMatrix(:,1), trimmedMatrix(:,3), trimmedMatrix(:,1), trimmedMatrix(:,4), trimmedMatrix(:,1), trimmedMatrix(:,5), 'LineWidth', 2);
hold on

plot(trimmedMatrixServo(:,1), trimmedMatrixServo(:,2), trimmedMatrixServo(:,1), trimmedMatrixServo(:,3), trimmedMatrixServo(:,1), trimmedMatrixServo(:,4), trimmedMatrixServo(:,1), trimmedMatrixServo(:,5));
legend('Intp Motor 1', 'Intp Motor 2', 'Intp Motor 3', 'Intp Motor 4', 'Servo Motor 1', 'Servo Motor 2', 'Servo Motor 3', 'Servo Motor 4');
grid on 
title('Motor Degrees vs Time (Interpolated vs Servo)');
xlabel('Time(s)');
ylabel('Motor Degrees');

hold off