%% Robot1
q1 = 2;
q2 = 3;
q3 = 5;
L1 = 2;
L2 = 5;
L3 = 4;
u = 30;
 
a0 = 0;
alfa0 = 0;
d1 = L1 + L1;
theta1 = q1;
 
a1 = 0;
alfa1 = u-90;
d2 = cos(u) * L2;
theta2 = q2;
 
a2 = 0;
alfa2 = -90;
d3 = sin(u) * L2;
theta3 = q3;
 
a3 = 0;
alfa3 = 90 - u;
d4 = L2 + L3;
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
q1 = 2;
q2 = 3;
q3 = 5;
q4 = 10;
q5 = 2;
L0 = 0.06;
L1 = 0.03;
L2 = 0.15;
L3 = 0.185;
L4 = 0.05;
L5 = 0.09;
 
a0 = 0;
alfa0 = 0;
d1 = -L0 -L1;
theta1 = q1;
 
a1 = 0;
alfa1 = -90;
d2 = 0;
theta2 = 90 + q2;
 
a2 = L2;
alfa2 = 180;
d3 = 0;
theta3 = q3 + 90;
 
a3 = L3;
alfa3 = 180;
d4 = 0;
theta4 = 90 + q4;  
 
a4 = 0;
alfa4 = 90;
d5 = L4 + L5;
theta5 = q5; 
 
T01 = custom_dh_link(alfa0, a0, d1, theta1);
T12 = custom_dh_link(alfa1, a1, d2, theta2);
T23 = custom_dh_link(alfa2, a2, d3, theta3);
T34 = custom_dh_link(alfa3, a3, d4, theta4);
T45 = custom_dh_link(alfa4, a4, d5, theta5);
T = T01 * T12 * T23 * T34 * T45; 
M{1} = T01;
M{2} = T12;
M{3} = T23;
M{4} = T34;
M{5} = T45;
 
DrawRobot(M);

a = [a0 a1 a2 a3 a4];
alpha = [alfa0 alfa1 alfa2 alfa3 alfa4];
d = [d1 d2 d3 d4 d5];
theta = [theta1 theta2 theta3 theta4 theta5];
bit_cupla = [0 0 0 0 1]; % 0 rotatie, 1 translatie
q = [q1 q2 q3 q4 q5];

figure;
for i=1:5
    L(i) = Link([theta(i) d(i) alpha(i) bit_cupla(i)],'modified');
end
roboArm = SerialLink(L,'robot','robot_sim');


roboArm.plot(q,'workspace',[-50 50 -50 50 0 50]);


function dh = custom_dh_link(alfa, a, d, theta)
    dh = rot(alfa, 'x') * transl([a; 0; 0]) * rot(theta, 'z') * transl([0; 0; d]);
end
function mat_transl = transl(v)
     mat_transl = [eye(3) v];
     mat_transl = [mat_transl; [0 0 0 1]];
end
 
function mat_rotatie = rot(nr, axa)
nr = deg2rad(nr);
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