%% Robot1
q1 = 2;
q2 = 3;
q3 = 5;
L1 = 1;
L2 = 1;
L3 = 1;
u = 30;
 
a0 = L1;
alfa0 = 0;
d1 = 0;
theta1 = q1;
 
a1 = 0;
alfa1 = 90;
d2 = L2;
theta2 = q2 + 90;
 
a2 = 0;
alfa2 = 90;
d3 = q3 + L3;
theta3 = 0;
 
T01 = custom_dh_link(alfa0, a0, d1, theta1);
T12 = custom_dh_link(alfa1, a1, d2, theta2);
T23 = custom_dh_link(alfa2, a2, d3, theta3);
T = T01 * T12 * T23; 
M{1} = T01;
M{2} = T12;
M{3} = T23;
 
DrawRobot(M);

%% Jacobian
a_vec = [a0 a1 a2];
alfa_vec = [alfa0 alfa1 alfa2];
d_vec = [d1 d2 d3];
theta_vec = [theta1 theta2 theta3];
bit_cupla_vec = [0 0 1];

J = calcul_jacobian(a_vec, alfa_vec, d_vec, theta_vec, bit_cupla_vec);

%% syms
syms q1;
syms q2;
syms q3;
syms L1;
syms L2;
syms L3;
syms a0;
syms a1;
syms a2;
syms alfa0;
syms alfa1;
syms alfa2;
syms d1;
syms d2;
syms d3;
syms theta1;
syms theta2;
syms theta3;

T01 = custom_dh_link(0, L1, 0, q1);
T12 = custom_dh_link(90, 0, L2, q2 + 90);
T23 = custom_dh_link(90, 0, q3 + L3, 0);

function dh = custom_dh_link(alfa, a, d, theta)
    dh = rot(alfa, 'x') * transl([a; 0; 0]) * rot(theta, 'z') * transl([0; 0; d]);
end
function mat_transl = transl(v)
     mat_transl = [eye(3) v];
     mat_transl = [mat_transl; [0 0 0 1]];
end
 
function mat_rotatie = rot(nr, axa)
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

function J = calcul_jacobian(a_vec, alfa_vec, d_vec, theta_vec, bit_cupla_vec)
syms a;
T_final = eye(4);
for i=1: length(bit_cupla_vec)
   T_final = T_final * custom_dh_link(alfa_vec(i), a_vec(i), d_vec(i), theta_vec(i)); 
end

J = zeros(6, length(bit_cupla_vec)) * a;
Ti = eye(4);

for i=1:length(bit_cupla_vec)
   Ti = Ti * custom_dh_link(alfa_vec(i), a_vec(i), d_vec(i), theta_vec(i));
   if (bit_cupla_vec(i) == 0)
      Jv = Ti(1:3, 3); 
      Jw = [0; 0; 0];
   else
      Jv = cross(Ti(1:3, 3) , T_final(1:3, 4) - Ti(1:3, 4));
      Jw = Ti(1:3, 3);
   end
   J(:, i) = [Jv; Jw];
end
end