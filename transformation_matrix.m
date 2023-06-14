%% Symbolic variable
clear
clc
syms theta1 theta2 theta3 theta4 theta5 theta6 ...
     d1 d2 d3 d4 d5 d6 ...
     a1 a2 a3 a4 a5 a6 ...
     alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 ...
     yoff1 yoff2 yoff3 yoff4 yoff5 yoff6 ...
     ytwist1 ytwist2 ytwist3 ytwist4 ytwist5 ytwist6

%% Transformation matrix
T01 = [[1 0 0 0]; [0 cosd(90) -sind(90) 0]; [0 sind(90) cosd(90) 0]; [0 0 0 1]] * ...
      [[cosd(90) 0 sind(90) 0]; [0 1 0 0]; [-sind(90) 0 cosd(90) 0]; [0 0 0 1]] * ...
      [[1 0 0 0]; [0 1 0 100]; [0 0 1 0]; [0 0 0 1]];
T12 = [[cos(theta1) -sin(theta1) 0 0]; [sin(theta1) cos(theta1) 0 0]; [0 0 1 0]; [0 0 0 1]] * ...
      [[1 0 0 0]; [0 1 0 0]; [0 0 1 180]; [0 0 0 1]] * ...
      [[1 0 0 240]; [0 1 0 0]; [0 0 1 0]; [0 0 0 1]] * ...
      [[cosd(-75) 0 sind(-75) 0]; [0 1 0 0]; [-sind(-75) 0 cosd(-75) 0]; [0 0 0 1]];
T23 = [[cos(theta2) -sin(theta2) 0 0]; [sin(theta2) cos(theta2) 0 0]; [0 0 1 0]; [0 0 0 1]] * ...
      [[1 0 0 300]; [0 1 0 0]; [0 0 1 0]; [0 0 0 1]] * ...
      [[cosd(128) 0 sind(128) 0]; [0 1 0 0]; [-sind(128) 0 cosd(128) 0]; [0 0 0 1]] * ...
      [[cosd(180) -sind(180) 0 0]; [sind(180) cosd(180) 0 0]; [0 0 1 0]; [0 0 0 1]];
T34 = [[1 0 0 0]; [0 1 0 0]; [0 0 1 -d3]; [0 0 0 1]];
T45 = [[cos(theta4) -sin(theta4) 0 0]; [sin(theta4) cos(theta4) 0 0]; [0 0 1 0]; [0 0 0 1]] * ...
      [[1 0 0 0]; [0 1 0 0]; [0 0 1 -200]; [0 0 0 1]] * ...
      [[1 0 0 0]; [0 cosd(90) -sind(90) 0]; [0 sind(90) cosd(90) 0]; [0 0 0 1]];
T56 = [[cos(theta5) -sin(theta5) 0 0]; [sin(theta5) cos(theta5) 0 0]; [0 0 1 0]; [0 0 0 1]] * ...
      [[1 0 0 20]; [0 1 0 0]; [0 0 1 0]; [0 0 0 1]] * ...
      [[1 0 0 0]; [0 cosd(90) -sind(90) 0]; [0 sind(90) cosd(90) 0]; [0 0 0 1]];
T6E = [[cos(theta6) -sin(theta6) 0 0]; [sin(theta6) cos(theta6) 0 0]; [0 0 1 0]; [0 0 0 1]] * ...
      [[1 0 0 20]; [0 1 0 0]; [0 0 1 0]; [0 0 0 1]];

T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;
T06 = T05 * T56;
T0E = T06 * T6E;

%% Extract Value
% quaternion
w = sqrt(1 + T0E(1,1) + T0E(2,2) + T0E(3,3)) /2;
i = (T0E(3,2) - T0E(2,3)) / (4*w);
j = (T0E(1,3) - T0E(3,1)) / (4*w);
k = (T0E(2,1) - T0E(1,2)) / (4*w);
% position
x = T0E(1,4);
y = T0E(2,4);
z = T0E(3,4);

%% Jacobian Matrix
J11 = diff(x, theta1);
J12 = diff(x, theta2);
J13 = diff(x, d3);
J14 = diff(x, theta4);
J15 = diff(x, theta5);
J16 = diff(x, theta6);

J21 = diff(y, theta1);
J22 = diff(y, theta2);
J23 = diff(y, d3);
J24 = diff(y, theta4);
J25 = diff(y, theta5);
J26 = diff(y, theta6);

J31 = diff(z, theta1);
J32 = diff(z, theta2);
J33 = diff(z, d3);
J34 = diff(z, theta4);
J35 = diff(z, theta5);
J36 = diff(z, theta6);

J41 = diff(w, theta1);
J42 = diff(w, theta2);
J43 = diff(w, d3);
J44 = diff(w, theta4);
J45 = diff(w, theta5);
J46 = diff(w, theta6);

J51 = diff(i, theta1);
J52 = diff(i, theta2);
J53 = diff(i, d3);
J54 = diff(i, theta4);
J55 = diff(i, theta5);
J56 = diff(i, theta6);

J61 = diff(j, theta1);
J62 = diff(j, theta2);
J63 = diff(j, d3);
J64 = diff(j, theta4);
J65 = diff(j, theta5);
J66 = diff(j, theta6);

J71 = diff(k, theta1);
J72 = diff(k, theta2);
J73 = diff(k, d3);
J74 = diff(k, theta4);
J75 = diff(k, theta5);
J76 = diff(k, theta6);


J = [
    [J11, J12, J13, J14, J15, J16];
    [J21, J22, J23, J24, J25, J26];
    [J31, J32, J33, J34, J35, J36];
    [J41, J42, J43, J44, J45, J46];
    [J51, J52, J53, J54, J55, J56];
    [J61, J62, J63, J64, J65, J66];
    [J71, J72, J73, J74, J75, J76]
    ];
