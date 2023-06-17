syms theta1 theta2 theta3 theta4 theta5 d6 theta7 theta8 theta9 theta10 theta11

A0_1 = [
    [-1 0 0 0];
    [0 -1 0 0];
    [0 0 1 148];
    [0 0 0 1]
    ];
A1_2 = [
    [cosd(theta2) -sind(theta2) 0 0];
    [0 0 -1 0];
    [sind(theta2) cosd(theta2) 0 0];
    [0 0 0 1]
    ];
A2_3 = [
    [0 -1 0 240.21];
    [1 0 0 0];
    [0 0 1 179.33];
    [0 0 0 1]
    ];
A3_4 = [
    [cosd(theta4) -sind(theta4) 0 0];
    [-0.259*sind(theta4) -0.259*cosd(theta4) -0.966 0];
    [0.966*sind(theta4) 0.966*cosd(theta4) -0.259 0];
    [0 0 0 1]
    ];
A4_5 = [
    [0 -1 0 150.32];
    [1 0 0 0];
    [0 0 1 0];
    [0 0 0 1]
    ];
A5_6 = [
    [1 0 0 0];
    [0 0.616 -0.788 -0.788*d6];
    [0 0.788 0.616 0.616*d6];
    [0 0 0 1]
    ];
A6_7 = [
    [1 0 0 0];
    [0 0 -1 -122.15];
    [0 1 0 0];
    [0 0 0 1]
    ];
A7_8 = [
    [cosd(0) -sind(0) 0 0];
    [0 0 -1 -485];
    [sind(0) cosd(0) 0 0];
    [0 0 0 1]
    ];
A8_9 = [
    [cosd(0) -sind(0) 0 0];
    [0 0 -1 0];
    [sind(0) cosd(0) 0 0];
    [0 0 0 1]
    ];
A9_10 = [
    [cosd(0) -sind(0) 0 0];
    [0 0 -1 0];
    [sind(0) cosd(0) 0 0];
    [0 0 0 1]
    ];
A10_11 = [
    [1 0 0 7];
    [0 1 0 0];
    [0 0 1 0];
    [0 0 0 1]
    ];

A0_2 = A0_1 * A1_2;
A0_3 = A0_2 * A2_3;
A0_4 = A0_3 * A3_4;
A0_5 = A0_4 * A4_5;
A0_6 = A0_5 * A5_6;
A0_7 = A0_6 * A6_7;
A0_8 = A0_7 * A7_8;
A0_9 = A0_8 * A8_9;
A0_10 = A0_9 * A9_10;
A0_11 = A0_10 * A10_11;


A0_11