%% Robot1
syms q1;
syms q2;
syms L1;
syms L2;
%q1 = 2;
%q2 = 3;
%L1 = 1;
%L2 = 1;
 
a0 = 0;
alfa0 = 0;
d1 = 0;
theta1 = q1;
 
ac1 = L1/2;
alfac1 = 0;
dc1 = 0;
thetac1 = 0;
 
a1 = L1;
alfa1 = 90;
d2 = 0;
theta2 = q2;

ac2 = L2/2;
alfac2 = 0;
dc2 = 0;
thetac2 = 0;
 
T01 = custom_dh_link(alfa0, a0, d1, theta1);
T1c1 = custom_dh_link(alfac1, ac1, dc1, thetac1);
T12 = custom_dh_link(alfa1, a1, d2, theta2);
T2c2 = custom_dh_link(alfac2, ac2, dc2, thetac2);

T0c1 = T01 * T1c1;
T0c2 = T01 * T12 * T2c2;

M{1} = T0c1;
M{2} = T0c2;

hc1 = T0c1(1, 4);
hc2 = T0c2(1, 4);
R0c1 = T0c1(1:3, 1:3);
R0c2 = T0c2(1:3, 1:3);

Jvc1_1 = cross(T01(1:3, 3), T0c1(1:3, 4) - T01(1:3, 4));
Jvc1_2 = [0; 0; 0];
Jwc1_1 = T01(1:3, 3);
Jwc1_2 = [0; 0; 0];
Jvc2_1 = cross(T01(1:3, 3), T0c2(1:3, 4) - T01(1:3, 4));
Jvc2_2 = cross(T12(1:3, 3), T0c2(1:3, 4) - T12(1:3, 4));
Jwc2_1 = T01(1:3, 3);
Jwc2_2 = T12(1:3, 3);

Jvc1 = [Jvc1_1 Jvc1_2];
Jwc1 = [Jwc1_1 Jwc1_2];
Jvc2 = [Jvc2_1 Jvc2_2];
Jwc2 = [Jwc2_1 Jwc2_2];

syms mc1 mc2;
%mc1 = 5;
%mc2 = 7;
Mc_1 = eye(3) * mc1;
Mc_2 = eye(3) * mc2;

syms ixci iyci izci;
%ixci = 0;
%iyci = 0;
%izci = 1;
Ic_1 = [ixci 0 0; 0 iyci 0; 0 0 izci];
Ic_2 = Ic_1;

Kvc1 = Jvc1' * Mc_1 * Jvc1;
Kvc2 = Jvc2' * Mc_2 * Jvc2;
Kwc1 = Jwc1' * R0c1 * Ic_1 * R0c1' * Jwc1;
Kwc2 = Jwc2' * R0c2 * Ic_2 * R0c2' * Jwc2;

Kvtot = Kvc1 + Kvc2;
Kwtot = Kwc1 + Kwc2;

D = Kvtot + Kwtot;

q = [q1; q2];
syms q1d q2d;
qd = [q1d; q2d];
C = zeros(2,2) * q1;
n = length(D(1,:));

for i = 1:n
    for j = 1:n
        for k = 1:n
            C(k,j) = C(k,j) + 1/2 * ((diff(D(k,j), q(i)) + diff(D(k,i), q(j)) - diff(D(i,j), q(k))) * qd(i));
        end
    end
end

Pc1 = mc1 * q1 * hc1;
Pc2 = mc2 * q2 * hc2;
P = Pc1 + Pc2;
G = [diff(P, q(1)); diff(P, q(2))];

%DrawRobot(M);

%% Simulink
% parametrii robotului
L1=1;L2=1;lc1=L1/2;lc2=L2/2;
m1=0.1;m2=0.1;I1=0.001;I2=0.001;
mc1 = 0.1; mc2 = 0.1;
g=-9.8;
ixci = 0; iyci = 0; izci = 1;

% punctul de echilibru
q1=0;q2=0;qd1=0;qd2=0;u1=0;u2=0;
 
d11=m1*lc1^2+m2*(L1^2+lc2^2+2*L1*lc2^2+2*L1*lc2*cos(q2))+I1+I2;
d12=m2*(lc2^2+L1*lc2*cos(q2))+I2;
d21=d12;
d22=m2*lc2^2+I2;
 
g1=(m1*lc1+m2*I1)*g*cos(q1)+m2*lc2*g*cos(q1+q2);
g2=m2*lc2*cos(q1+q2);
 
c11=-2*m2*L1*lc2*sin(q2)*qd2;
c12=-m2*L1*lc2*sin(q2)*qd2;
c21=m2*L1*lc2*sin(q2)*qd1;
c22=0;
 
%D=[d11 d12;d21 d22];
%C=[c11 c12;c21 c22];
%G=[g1;g2];
D = eval(D);
C = eval(C);
G = eval(G);
 
u = [u1; u2];
q_dd = inv(D) - (u-C-G);

x = [q1; q2; qd1; qd2];
f = [qd1; qd2; q_dd(1); q_dd(2)];
A = jacobian(f,x);
B = jacobian(f,u);

% A si B dupa liniarizarea in punctul de echilibru
A = eval(A);
B = eval(B);
 
% gain ul K al regulatorului prin alocarea de poli
K = place(A,B, [-11 11 -4 -4]);

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