%% Robot1
q1 = 2;
q2 = 3;
q3 = 5;
q4 = 10;
L1 = 2;
L2 = 5;
 
a0 = 0;
alfa0 = 0;
d1 = L1;
theta1 = q1;
 
a1 = 0;
alfa1 = 0;
d2 = q2;
theta2 = 0;
 
a2 = 0;
alfa2 = -90;
d3 = q3;
theta3 = 0;
 
a3 = 0;
alfa3 = 0;
d4 = L2;
theta4 = 0;  
 
 
T01 = custom_dh_link(alfa0, a0, d1, theta1);
T12 = custom_dh_link(alfa1, a1, d2, theta2);
T23 = custom_dh_link(alfa2, a2, d3, theta3);
T34 = custom_dh_link(alfa3, a3, d4, theta4);
T = T01 * T12 * T23 * T34; 
M{1} = T01;
M{2} = T12;
M{3} = T23;
M{4} = T34;
 
DrawRobot(M);
 
%% Robot2
q1 = 3;
q2 = 1;
q3 = 1;
L0 = 2;
L1 = 2;
L3 = 1;
 
a0 = 0;
alfa0 = 0; 
d1 = L0;
theta1 = q1;
 
a1 = L1;
alfa1 = 0;
d2 = 0;
theta2 = q2;
 
a2 = L2;
alfa2 = -180; %sau 180;
d3 = q3;
theta3 = 0;
 
a3 = 0;
alfa3 = 0;
d4 = L3;
theta4 = 0;
 
T01 = custom_dh_link(alfa0, a0, d1, theta1);
T12 = custom_dh_link(alfa1, a1, d2, theta2);
T23 = custom_dh_link(alfa2, a2, d3, theta3);
T34 = custom_dh_link(alfa3, a3, d4, theta4);
T = T01 * T12 * T23 * T34; 
M{1} = T01;
M{2} = T12;
M{3} = T23;
M{4} = T34;
 
DrawRobot(M);
 
%% Robot3
q1 = 3;
q2 = 1;
q3 = 1;
q4 = 2;
L1 = 2;
L2 = 2;
L3 = 1;
 
a0 = 0;
alfa0 = 0; 
d1 = L1;
theta1 = q1;
 
a1 = 0;
alfa1 = 0;
d2 = q2;
theta2 = 0;
 
a2 = L2;
alfa2 = 0;
d3 = 0;
theta3 = 270 + q3;
 
a3 = 0;
alfa3 = -90;
d4 = q4;
theta4 = 0;
 
a4 = 0;
alfa4 = 0;
d5 = L3;
theta5 = 0; 
 
T01 = custom_dh_link(alfa0, a0, d1, theta1);
T12 = custom_dh_link(alfa1, a1, d2, theta2);
T23 = custom_dh_link(alfa2, a2, d3, theta3);
T34 = custom_dh_link(alfa3, a3, d4, theta4);
T = T01 * T12 * T23 * T34; 
M{1} = T01;
M{2} = T12;
M{3} = T23;
M{4} = T34;
 
DrawRobot(M);
 
function dh = custom_dh_link(alfa, a, d, theta)
    dh = rot(alfa, 'x') * transl([a; 0; 0]) * rot(theta, 'z') * transl([0; 0; d]);
end
function mat_transl = transl(v)
     mat_transl = [eye(3) v];
     mat_transl = [mat_transl; [0 0 0 1]];
end
 
function mat_rotatie = rot(nr, axa)
%nr = deg2rad(nr);
switch axa
    case 'x'
        mat_rotatie = [1 0 0 0; 0 cos(nr) -sin(nr) 0; 0 sin(nr) cos(nr) 0; 0 0 0 1];
    case 'y'
        mat_rotatie = [cos(nr) 0 sin(nr) 0; 0 1 0 0; -sin(nr) 0 cos(nr) 0; 0 0 0 1;];
    case 'z'
        mat_rotatie = [cos(nr) -sin(nr) 0 0; sin(nr) cos(nr) 0 0; 0 0 1 0; 0 0 0 1;];
end
end
 
function t_inv = transf_inv(T)
     t_inv = eye(4);
     t_inv(1:3,1:3) = T(1:3,1:3)';
     t_inv(1:3,4) = T(1:3,1:3)'*-T(1:3,4);
end