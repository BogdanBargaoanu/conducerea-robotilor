syms q1 q2 dq1 dq2 u1 u2

% parametrii robotului
l1=1;l2=1;lc1=l1/2;lc2=l2/2;
m1=0.1;m2=0.1;I1=0.001;I2=0.001;
g=-9.8;

d11=m1*lc1^2+m2*(l1^2+lc2^2+2*l1*lc2^2+2*l1*lc2*cos(q2))+I1+I2;
d12=m2*(lc2^2+l1*lc2*cos(q2))+I2;
d21=d12;
d22=m2*lc2^2+I2;

g1=(m1*lc1+m2*I1)*g*cos(q1)+m2*lc2*g*cos(q1+q2);
g2=m2*lc2*cos(q1+q2);

c11=-2*m2*l1*lc2*sin(q2)*dq2;
c12=-m2*l1*lc2*sin(q2)*dq2;
c21=m2*l1*lc2*sin(q2)*dq1;
c22=0;

D=[d11 d12;d21 d22];
C=[c11 c12;c21 c22];
G=[g1;g2];

x = [q1; q2; dq1; dq2];
u = [u1; u2];
q_dd = inv(D) - (u-C-G);
f = [dq1; dq2; q_dd(1); q_dd(2)];
A = jacobian(f,x);
B = jacobian(f,u);
% punctul de echilibru
q1=0;q2=0;dq1=0;dq2=0;u1=0;u2=0;
A = eval(A);
B = eval(B);

% A si B dupa liniarizarea in punctul de echilibru


% gain ul K al regulatorului prin alocarea de poli
K = place(A,B, [-5 -7 -5 -3]);
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