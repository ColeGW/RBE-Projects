A = csvread("Lab_3/cubicJoint.csv");
B = csvread("Lab_3/cubicTask.csv");
C = csvread("Lab_3/noInterp.csv");

XA = A(:, 1);
YA = A(:, 2);
ZA = A(:, 3);

XB = B(:, 1);
YB = B(:, 2);
ZB = B(:, 3);

XC = C(:, 1);
YC = C(:, 2);
ZC = C(:, 3);

     figure(1);
     scatter3(XA, YA, ZA, '*', 'DisplayName', 'XYZ cubicJoint');
     hold on
    scatter3(XB, YB, ZB, '*', 'DisplayName', 'XYZ cubicTask');
    scatter3(XC, YC, ZC, '*', 'DisplayName', 'XYZ No Interpolation');
    title('End effector XYZ values (Task vs Joint vs None)')
     xlabel('X Value (mm)')
     ylabel('Y Value (mm)')
     zlabel('Z Value (mm)')
     %writematrix([Ya4, Ya5, Ya6], 'noInterp.csv');
     hold off
     legend