clear
clc

syms L1 L2 L3 L4 L5 L6 Q1 Q2 Q3 Q4 Q5 Q6
alpha = [90, 0, -90, 90, -90, 0];   % this is the alpha value for all  the link
a = [0, 150, 0, 0, 0, 0];           % Length of the Link
d = [45, 0, 0, 150, 0, 0];          % Offset
Q = [Q1, Q2, Q3, Q4, Q5, Q6];       % joint angle variation

% Transformation Matrices
for i = 1:6
switch i
    case 1
        T01= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alpha(1,i)),sind(alpha(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alpha(1,i)),-sind(alpha(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alpha(1,i)),cosd(alpha(1,i)),d(1,i);0,0,0,1];
    case 2
        T12= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alpha(1,i)),sind(alpha(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alpha(1,i)),-sind(alpha(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alpha(1,i)),cosd(alpha(1,i)),d(1,i);0,0,0,1];
        T02=T01*T12;
    case 3
        T23= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alpha(1,i)),sind(alpha(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alpha(1,i)),-sind(alpha(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alpha(1,i)),cosd(alpha(1,i)),d(1,i);0,0,0,1];
        T03=T02*T23;
    case 4
        T34= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alpha(1,i)),sind(alpha(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alpha(1,i)),-sind(alpha(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alpha(1,i)),cosd(alpha(1,i)),d(1,i);0,0,0,1];
        T04=T03*T34;
    case 5
        T45= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alpha(1,i)),sind(alpha(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alpha(1,i)),-sind(alpha(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alpha(1,i)),cosd(alpha(1,i)),d(1,i);0,0,0,1];
        T05=T04*T45;
    case 6
        T56= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alpha(1,i)),sind(alpha(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alpha(1,i)),-sind(alpha(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alpha(1,i)),cosd(alpha(1,i)),d(1,i);0,0,0,1];
        T06=T05*T56;
end
end
