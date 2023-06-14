syms theta1 theta2 theta3 theta4 theta5 theta6

A0_1 = [
    [cosd(theta1) -sind(theta1) 0 0];
    [sind(theta1) cosd(theta1) 0 0];
    [0 0 1 45];
    [0 0 0 1]
    ];
A1_2 = [
    [cosd(theta2) -sind(theta2) 0 0];
    [0 0 -1 0];
    [sind(theta2) cosd(theta2) 0 0];
    [0 0 0 1]
    ];
A2_3 = [
    [cosd(theta3) -sind(theta3) 0 150];
    [sind(theta3) cosd(theta3) 0 0];
    [0 0 1 0];
    [0 0 0 1]
    ];
A3_4 = [
    [cosd(theta4) -sind(theta4) 0 0];
    [0 0 1 150];
    [-sind(theta4) -cosd(theta4) 0 0];
    [0 0 0 1]
    ];
A4_5 = [
    [cosd(theta5) -sind(theta5) 0 0];
    [0 0 -1 0];
    [sind(theta5) cosd(theta5) 0 0];
    [0 0 0 1]
    ];
A5_6 = [
    [cosd(theta6) -sind(theta6) 0 0];
    [0 0 1 0];
    [-sind(theta6) -cosd(theta6) 0 0];
    [0 0 0 1]
    ];

A0_2 = A0_1 * A1_2;
A0_3 = A0_2 * A2_3;
A0_4 = A0_3 * A3_4;
A0_5 = A0_4 * A4_5;
A0_6 = A0_5 * A5_6;

A0_6